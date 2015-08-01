var n = 0;

var newMeanSec = 0;
var oldMeanSec = 0;

var newTime;
var oldTime;

module.exports = {

    tick: function() {

        n++;

        if (n == 1) {

            oldTime = new Date();
            newTime = new Date();

        }

        else {

            oldTime = newTime;
            newTime = new Date();

            var newSec = -(oldTime - newTime) / 1000;

            newMeanSec = oldMeanSec + (newSec - oldMeanSec) / n;
            oldMeanSec = newMeanSec;

        }

    },

    print: function() {

        var hz = 1 / newMeanSec;
        console.log('FPS: ' + hz);

    }

}