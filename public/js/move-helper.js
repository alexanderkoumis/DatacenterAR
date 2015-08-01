function MoveHelper(moveScale, moveDecay) {

    this.scale = moveScale;
    this.decay = moveDecay;

    this.currentKey = 0;
    this.velocity = new THREE.Vector3();

    this.moveForward = false;
    this.moveLeft = false;
    this.moveRight = false;
    this.moveBackward = false;

    this.timeCurr = 0.0;
    this.timePrev = 0.0;
    this.delta = 0.0;

    this.position = new THREE.Vector3(0.0, 0.0, 0.0);
    this.velocity = new THREE.Vector3(0.0, 0.0, 0.0);
    this.orientation = new THREE.Quaternion(0.0, 0.0, 0.0, 0.0);
}

MoveHelper.prototype.move = function(object) {

    this.timePrev = this.timeCurr;
    this.timeCurr = performance.now();
    this.delta = ( this.timeCurr - this.timePrev ) / 1000;

    if ( this.moveForward ) this.velocity.z -= parseFloat(this.scale) * parseFloat(this.delta);
    if ( this.moveBackward ) this.velocity.z += parseFloat(this.scale) * parseFloat(this.delta);

    if ( this.moveLeft ) this.velocity.x -= parseFloat(this.scale) *  parseFloat(this.delta);
    if ( this.moveRight ) this.velocity.x += parseFloat(this.scale) * parseFloat(this.delta);

    this.velocity.x -= this.velocity.x * this.decay * this.delta;
    this.velocity.z -= this.velocity.z * this.decay * this.delta;

    object.position.x += this.velocity.x;
    object.position.y += this.velocity.y;
    object.position.z += this.velocity.z;

}