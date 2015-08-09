var textureDict = {};
var focalLenScaled = {};

var deviceImages = [
    {u: 1, list: ['textures/_1U_front_server_pure_m.png']},
    {u: 2, list: ['textures/_2U_front_server_hp_storageworks.png',
                  'textures/_2U_front_server_pure_fa.png',
                  'textures/_2U_front_shelf_pure_fa.png']},
    {u: 4, list: ['textures/_4U_front_shelf_hp_storageworks.png']}
];


var initCamPosition = {
    x: 0,
    y: 750,
    z: -150
};

var initLightPosition = {
    x: 0,
    y: 2000,
    z: 3000
};

var focalLenOrig = {
    x: 525.0,
    y: 525.0
};

var canvasDims = {
    width: 320,
    height: 256,
};

var rackDims = {
    rows: 26,
    cols: 43
};

var uDimsMm = {
    space: 5,
    width: 241,
    height: 44,
    length: 465
};

var poseScale = 1000;

var uWidthPlusSpacer = uDimsMm.width + uDimsMm.space;
var rackWidthHalf = rackDims.cols * uWidthPlusSpacer/2;

var clientSet = false;
var client = new BinaryClient('ws://' + window.location.hostname + ':9000');

var video = document.getElementById('video');
var canvas = document.getElementById('canvas');
var container = document.getElementById('container');
var canvasContext = canvas.getContext('2d');

var scene = new THREE.Scene();
var renderer = new THREE.WebGLRenderer({ alpha: true });
var cameraPerspective = new THREE.PerspectiveCamera( 70, window.innerWidth / window.innerHeight, 1, 50000 );
var light = new THREE.PointLight();

var imageWorker = new Worker('js/image-worker.js');

var backgroundTexture = new THREE.Texture(canvas);
var backgroundGeometry = new THREE.PlaneGeometry(2, 2, 0);
var backgroundMaterial = new THREE.MeshBasicMaterial({map: backgroundTexture});
var backgroundMesh = new THREE.Mesh(backgroundGeometry, backgroundMaterial);
var backgroundScene = new THREE.Scene();
var backgroundCamera = new THREE.Camera();

var gridHelper = new THREE.GridHelper( 10000, 100 );
var moveHelper = new MoveHelper(3000.0, 10.0);
// var axisArrows = new AxisArrows(500);

reallyGetUserMedia(camSelection, video);

video.width = canvasDims.width;
video.height = canvasDims.height;

canvas.width = canvasDims.width;
canvas.height = canvasDims.height;

renderer.setClearColor( 0x000000, 1 );
renderer.sortObjects = true;
renderer.setPixelRatio( window.devicePixelRatio );
renderer.setSize( window.innerWidth, window.innerHeight );
container.appendChild( renderer.domElement );

backgroundMesh.material.depthTest = false;
backgroundMesh.material.depthWrite = false;
backgroundMesh.material.map.needsUpdate = true;

backgroundScene.add(backgroundCamera);
backgroundScene.add(backgroundMesh);

cameraPerspective.position.set( initCamPosition.x, initCamPosition.y, initCamPosition.z);
light.position.set( initLightPosition.x, initLightPosition.y, initLightPosition.z );

gridHelper.setColors( 0x0000ff, 0x808080 );
gridHelper.material.opacity = 0.25;
gridHelper.material.transparent = true;

// axisArrows.addToScene(scene);
scene.add( light );
scene.add( new THREE.PointLightHelper( light, 10 ) );
scene.add( gridHelper );

var greenBoxMaterial = new THREE.MeshBasicMaterial( { color: 0x00ffff, wireframe: true } );
var greenBoxGeometry = new THREE.BoxGeometry( 500, 500, 500 );
var greenBoxMesh = new THREE.Mesh( greenBoxGeometry, greenBoxMaterial );

// Preload devices meshes
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

// Draw devices
for ( var col = 0; col < rackDims.cols; ++col ) {

    var row = 0;

    do {

        var deviceUListIdx = Math.floor(Math.random() * 100) % deviceImages.length;
        var deviceU = deviceImages[deviceUListIdx].u;

        if ((row + deviceU) > rackDims.rows) {
            break;
        }

        var deviceImageList = deviceImages[deviceUListIdx].list;
        var deviceImageListNum = Math.floor(Math.random()) % deviceImageList.length;
        var deviceImagePath = deviceImageList[deviceImageListNum];

        var boxMesh = textureDict[deviceImagePath].clone();

        boxMesh.position.x = col * uWidthPlusSpacer - rackWidthHalf;
        boxMesh.position.y = row * uDimsMm.height + uDimsMm.space;
        boxMesh.position.z = -2000;

        scene.add ( boxMesh );

        row += deviceU;

    } while (row < rackDims.rows);

}

// Event listeners
imageWorker.onmessage = function(image) {

    if (clientSet && userMediaSet) {

        client.send(image.data, {
            id: getVideoId(),
            width: canvasDims.width,
            height: canvasDims.height,
            channels: 1,
            focalLen: focalLenScaled,
            length: image.data.length,
            interface: 'rawTo8UC4'
        });

    }
}

window.addEventListener('resize', function () {

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

video.addEventListener('playing', function getVideoSize() {

    var videoDims = {
        width: video.videoWidth,
        height: video.videoHeight
    };

    var videoScale = {
        width: canvasDims.width / videoDims.width,
        height: canvasDims.height / videoDims.height
    };
    
    focalLenScaled = {
        x: focalLenOrig.x * videoScale.width,
        y: focalLenOrig.y * videoScale.height
    };

    video.removeEventListener('playing', getVideoSize, false);

    userMediaSet = true;

}, false);

function dataURItoBlob(dataURI) {

    var byteString = atob(dataURI.split(',')[1]);
    var ab = new ArrayBuffer(byteString.length);
    var ia = new Uint8Array(ab);
    for (var i = 0; i < byteString.length; i++) {
        ia[i] = byteString.charCodeAt(i);
    }
    return new Blob([ab]);

}

function adjustPose(object, poseArr) {

    var poseX = parseFloat(poseArr[0]) * poseScale + initCamPosition.x;
    var poseY = parseFloat(poseArr[1]) * poseScale + initCamPosition.y;
    var poseZ = parseFloat(poseArr[2]) * poseScale + initCamPosition.z;

    object.position.set( poseX, poseY, poseZ );
    var quaternion = new THREE.Quaternion(poseArr[3], -poseArr[4], poseArr[5], poseArr[6]);
    object.rotation.setFromQuaternion(quaternion.normalize(), 'XYZ');

}

function switchCamera() {

    camSelection = (camSelection) ? 0 : 1;
    camSelection = (videoSources.length > 1) ? camSelection : 0;
    reallyGetUserMedia(camSelection, video);
    console.log('camera: ' + camSelection);

}

(function loop() {

    if (userMediaSet) {
        var image8UC4 = canvasContext.getImageData(0, 0, canvasDims.width, canvasDims.height);
        if (image8UC4.data) {
            imageWorker.postMessage(image8UC4);
        }
    }

    backgroundMesh.material.map.needsUpdate = true;

    canvasContext.drawImage(video, 0, 0, canvasDims.width, canvasDims.height);

    moveHelper.move(cameraPerspective);

    renderer.autoClear = false;
    renderer.clear();
    renderer.render( backgroundScene, backgroundCamera );
    renderer.clearDepth();
    renderer.render( scene, cameraPerspective );

    requestAnimationFrame( loop );

})();

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
    }

    else {
        console.log('stream: ' + stream)
    }

});
