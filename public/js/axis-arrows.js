function AxisArrows(length) {
    this.length = length;

    var xDir = new THREE.Vector3( 1, 0, 0 );
    var yDir = new THREE.Vector3( 0, 1, 0 );
    var zDir = new THREE.Vector3( 0, 0, 1 );

    var xHex = 0xff0000;
    var yHex = 0x00ff00;
    var zHex = 0x0000ff;

    var origin = new THREE.Vector3( 0, 0, 0 );  

    this.xArrow = new THREE.ArrowHelper( xDir, origin, length, xHex );
    this.yArrow = new THREE.ArrowHelper( yDir, origin, length, yHex );
    this.zArrow = new THREE.ArrowHelper( zDir, origin, length, zHex );
}

AxisArrows.prototype.adjustPose = function(object, poseArr) {
    object.position.x = parseFloat(poseArr[0]);
    object.position.y = parseFloat(poseArr[1]);
    object.position.z = parseFloat(poseArr[2]);
    // var quaternion = new THREE.Quaternion(poseArr[3], poseArr[5], -poseArr[4], poseArr[6]);
    // object.rotation.setFromQuaternion(quaternion.normalize(), 'XYZ');
}

AxisArrows.prototype.update = function(poseArr) {
    adjustPose(this.xArrow, poseArr);
    adjustPose(this.yArrow, poseArr);
    adjustPose(this.zArrow, poseArr);
}

AxisArrows.prototype.addToScene = function(scene) {
	scene.add( this.xArrow );
	scene.add( this.yArrow );
	scene.add( this.zArrow );
}