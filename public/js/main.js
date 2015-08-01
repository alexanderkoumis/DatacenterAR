var settings = {
    canvasWidth: 192,
    canvasHeight: 128,
    imageStep: 30,
    binarySrv: 'ws://' + window.location.hostname + ':9000'
};

var picNum = -1;
var picsPerInterface = 100;
var startTime;
var endTime;

var client = new BinaryClient(settings.binarySrv);

var video = document.getElementById('video');
var canvas = document.getElementById('canvas');
var container = document.getElementById('container');

var context;
var scene;
var renderer;
var cameraPerspective;
var cameraTracked;
var light;

var backgroundTexture;
var backgroundMesh;
var backgroundScene;
var backgroundCamera;

var videoSources = [];
var camSelection = 0;

var box;
var axisArrows = new AxisArrows(50);
var moveHelper;

var poseSet = false;
var clientSet = false;
var userMediaSet = false;

var imageWorker = new Worker('image-worker.js');

var initCamPosition = {
    x: 0,
    y: 1000,
    z: 5000
}

var uDimsMm = {
    space: 0,
    width: 241,
    height: 44,
    length: 465
}

var rows = 26;
var cols = 43;

var uWidthPlusSpacer = uDimsMm.width + uDimsMm.space;
var rackWidthHalf = cols * uWidthPlusSpacer/2;

var deviceImages = [
    {u: 1, list: ['textures/_1U_front_server_pure_m.png']},
    {u: 2, list: ['textures/_2U_front_server_hp_storageworks.png',
                  'textures/_2U_front_server_pure_fa.png',
                  'textures/_2U_front_shelf_pure_fa.png']},
    {u: 4, list: ['textures/_4U_front_shelf_hp_storageworks.png']}
]

var textureDict = {};
var seenHeights = [];

(function () {

    deviceImages.forEach(function(deviceImageObj) {

        var uNum = deviceImageObj.u;
        var deviceImageLists = deviceImageObj.list;

        deviceImageLists.forEach(function(deviceImageStr) {

            var boxMaterialArray = [];
            var textureFront = THREE.ImageUtils.loadTexture(deviceImageStr);

            boxMaterialArray.push( new THREE.MeshBasicMaterial( { color: 0xbdbbbb } ) );     // +x
            boxMaterialArray.push( new THREE.MeshBasicMaterial( { color: 0xbdbbbb } ) );     // -x
            boxMaterialArray.push( new THREE.MeshBasicMaterial( { color: 0xbdbbbb } ) );     // +y
            boxMaterialArray.push( new THREE.MeshBasicMaterial( { color: 0xbdbbbb } ) );     // -y
            boxMaterialArray.push( new THREE.MeshLambertMaterial( { map: textureFront } ) ); // +z
            boxMaterialArray.push( new THREE.MeshLambertMaterial( { map: textureFront } ) ); // -z
            var boxMaterials = new THREE.MeshFaceMaterial( boxMaterialArray );

            var deviceDimsMm = uDimsMm;

            // if (uNum >= 2) {
            //     deviceDimsMm.height *= uNum;
            // }

            var boxGeometry = new THREE.BoxGeometry( deviceDimsMm.width, deviceDimsMm.height, deviceDimsMm.length);
            var boxMesh = new THREE.Mesh( boxGeometry, boxMaterials );
    
            textureDict[deviceImageStr] = boxMesh;

            // THREE.MeshBasicMaterial( { color: 0x00ffff, wireframe: true } );
            // THREE.MeshBasicMaterial( { color: 0xff0000, blending: THREE.AdditiveBlending } );
            // THREE.MeshLambertMaterial( { color: 0xffffff, shading: THREE.FlatShading, overdraw: true } );
            // THREE.MeshLambertMaterial( { color: 0xffffff, shading: THREE.SmoothShading, overdraw: true } );
            // THREE.MeshDepthMaterial( { overdraw: true } );
            // THREE.MeshNormalMaterial( { overdraw: true } );
            // THREE.MeshBasicMaterial( { map: texture } );
            // THREE.MeshLambertMaterial( { map: texture } );
            // THREE.MeshBasicMaterial( { map: texture } );

        });

    });

})();

function drawBoxes() {

    for ( var col = 0; col < cols; ++col ) {

        var row = 0;

        do {

            var deviceUListIdx = Math.floor(Math.random() * 100) % deviceImages.length;
            var deviceU = deviceImages[deviceUListIdx].u;

            if ((row + deviceU) > rows) {
                break;
            }

            var deviceImageList = deviceImages[deviceUListIdx].list;
            var deviceImageListNum = Math.floor(Math.random()) % deviceImageList.length;
            var deviceImagePath = deviceImageList[deviceImageListNum];

            var boxMesh = textureDict[deviceImagePath].clone();

            boxMesh.position.x = col * uWidthPlusSpacer - rackWidthHalf;
            boxMesh.position.y = row * uDimsMm.height;

            scene.add ( boxMesh );

            row += deviceU;

        } while (row < rows);

    }

}

document.addEventListener('DOMContentLoaded', function(){

    (function() {

        MediaStreamTrack.getSources(function (sourceInfos) {

            for (var i = 0; i != sourceInfos.length; ++i) {
                var sourceInfo = sourceInfos[i];
                if (sourceInfo.kind === 'video') {
                    console.log(sourceInfo.id, sourceInfo.facing, sourceInfo.label || 'camera');
                    videoSources.push(sourceInfo.id);
                } else {
                    console.log('Some other kind of source: ' + sourceInfo.kind);
                }
            }
            console.log("sources", videoSources);

        });

        if (typeof videoSources[1] === 'undefined') {
            camSelection = 0;
        }
        else {
            if (videoSources[0].facing == 'environment') camSelection = 0;
            if (videoSources[1].facing == 'environment') camSelection = 1;
        }
        console.log('cam selection: ' + camSelection);

        reallyGetUserMedia(camSelection);

    })();

    video.width = settings.canvasWidth;
    video.height = settings.canvasHeight;

    canvas.width = settings.canvasWidth;
    canvas.height = settings.canvasHeight;
    context = canvas.getContext('2d');

    context.drawImage(video, 0, 0, settings.canvasWidth, settings.canvasHeight);

    renderer = new THREE.WebGLRenderer({ alpha: true });
    renderer.setClearColor( 0x000000, 1 );
    renderer.sortObjects = true;
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setSize( window.innerWidth, window.innerHeight );
    container.appendChild( renderer.domElement );

    cameraPerspective = new THREE.PerspectiveCamera( 70, window.innerWidth / window.innerHeight, 1, 50000 );

    cameraPerspective.position.x = initCamPosition.x;
    cameraPerspective.position.y = initCamPosition.y;
    cameraPerspective.position.z = initCamPosition.z;

    scene = new THREE.Scene();

    // Background
    backgroundTexture = new THREE.Texture(canvas);
    backgroundTexture.needsUpdate = true;

    var geometry = new THREE.PlaneGeometry(2, 2, 0);
    var material = new THREE.MeshBasicMaterial({map: backgroundTexture});

    backgroundMesh = new THREE.Mesh(geometry, material);

    backgroundMesh.material.depthTest = false;
    backgroundMesh.material.depthWrite = false;
    backgroundMesh.material.map.needsUpdate = true;

    backgroundScene = new THREE.Scene();
    backgroundCamera = new THREE.Camera();

    backgroundScene.add(backgroundCamera);
    backgroundScene.add(backgroundMesh);

    // Light
    light = new THREE.PointLight();
    light.position.set( 0, 2000, 3000 );
    scene.add( light );
    scene.add( new THREE.PointLightHelper( light, 100 ) );

    axisArrows.addToScene(scene);

    moveHelper = new MoveHelper(3000.0, 10.0);

    drawBoxes();

    // Helper grid
    var helper = new THREE.GridHelper( 10000, 100 );
    helper.setColors( 0x0000ff, 0x808080 );
    helper.material.opacity = 0.25;
    helper.material.transparent = true;
    scene.add( helper );

    var boxGeometry = new THREE.BoxGeometry(500, 500, 500);
    var boxMaterial = new THREE.MeshBasicMaterial( {color: 0x00ff00} );
    box = new THREE.Mesh( boxGeometry, boxMaterial );
    box.position.y = 50;
    scene.add( box );

    imageWorker.onmessage = function(image) {

        if (clientSet) {
            client.send(image.data, {
                width: settings.canvasWidth,
                height: settings.canvasHeight,
                channels: 1,
                step: settings.imageStep,
                length: image.data.length,
                interface: 'rawTo8UC4'
            });
        }

    }

    window.addEventListener( 'resize', function () {

        cameraPerspective.aspect = window.innerWidth / window.innerHeight;
        cameraPerspective.updateProjectionMatrix();
        renderer.setSize( window.innerWidth, window.innerHeight );

    }, false );

    window.addEventListener('keydown', function(event) {

        switch (event.keyCode) {
            case 38: moveHelper.moveForward = true; break;  // up arrow
            case 87: moveHelper.moveForward = true; break;  // w
            case 37: moveHelper.moveLeft = true; break;     // left
            case 65: moveHelper.moveLeft = true; break;     // a
            case 40: moveHelper.moveBackward = true; break; // down
            case 83: moveHelper.moveBackward = true; break; // s
            case 39: moveHelper.moveRight = true; break;    // right
            case 68: moveHelper.moveRight = true; break;    // d
        }

    }, false);

    window.addEventListener('keyup', function(event) {

        switch (event.keyCode) {
            case 38: moveHelper.moveForward = false; break;  // up
            case 87: moveHelper.moveForward = false; break;  // w
            case 37: moveHelper.moveLeft = false; break;     // left
            case 65: moveHelper.moveLeft = false; break;     // a
            case 40: moveHelper.moveBackward = false; break; // down
            case 83: moveHelper.moveBackward = false; break; // s
            case 39: moveHelper.moveRight = false; break;    // right
            case 68: moveHelper.moveRight = false; break;    // d
        }

    }, false);

    loop();
});

function reallyGetUserMedia(camSelect) {

    navigator.getUserMedia = (navigator.getUserMedia ||
                              navigator.webkitGetUserMedia ||
                              navigator.mozGetUserMedia ||
                              navigator.msGetUserMedia);

    var constraints = {
        video: { optional: [{ sourceId: videoSources[camSelect] }] },
        audio: false
    };

    navigator.getUserMedia(constraints, function(stream) {

        video.src = window.URL.createObjectURL(stream);
        video.play();
        userMediaSet = true;

    }, function(error) { console.log('error: ' + error); });
}

function dataURItoBlob(dataURI) {

    var byteString = atob(dataURI.split(',')[1]);
    var ab = new ArrayBuffer(byteString.length);
    var ia = new Uint8Array(ab);
    for (var i = 0; i < byteString.length; i++) {
        ia[i] = byteString.charCodeAt(i);
    }
    return new Blob([ab]);

}

function sendImage() {

    if (userMediaSet) {
        var image8UC4 = context.getImageData(0, 0, settings.canvasWidth, settings.canvasHeight);
        if (image8UC4.data) {
            imageWorker.postMessage(image8UC4);
        }
    }

}

function adjustPose(object, poseArr) {

    object.position.x = parseFloat(poseArr[0]);
    object.position.y = parseFloat(poseArr[1]);
    object.position.z = parseFloat(poseArr[2]);
    var quaternion = new THREE.Quaternion(poseArr[3], poseArr[5], -poseArr[4], poseArr[6]);
    object.rotation.setFromQuaternion(quaternion.normalize(), 'XYZ');

}

function loop() {

    sendImage();

    backgroundMesh.material.map.needsUpdate = true;

    context.drawImage(video, 0, 0, settings.canvasWidth, settings.canvasHeight);

    moveHelper.move(cameraPerspective);

    renderer.autoClear = false;
    renderer.clear();
    renderer.render( backgroundScene , backgroundCamera );
    renderer.clearDepth();
    renderer.render( scene, cameraPerspective );

    requestAnimationFrame( loop );

}

function switchCamera() {

  camSelection = (camSelection) ? 0 : 1;
  camSelection = (videoSources.length > 1) ? camSelection : 0;
  reallyGetUserMedia(camSelection);
  console.log('camera: ' + camSelection);

}

client.on('open', function () {

    console.log('its open');
    clientSet = true;

});

client.on('stream', function (stream, meta) {

    if (meta.type == 'pose') {

        var poseArr = meta.data.split(',');

        if ((poseArr.length < 0) || (poseArr.length > 7)) {
            console.log('Error: ' + poseArr.length + ' is an invalid number of elements');
            return;
        }

        adjustPose(cameraPerspective, poseArr);
        // adjustPose(box, poseArr);

        poseSet = true;
    }

    else {
        console.log('stream: ' + stream)
    }
});
