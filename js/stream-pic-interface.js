var fs         = require('fs');
var lz4        = require('lz4')
var pako       = require('pako');
var PNG        = require('pngjs').PNG;
var streamBuff = require('stream-buffers');

var picNum = 0;

function errorLog(message) {

    console.log('Error: ' + message);
    process.exit();

}

module.exports = {

    rawTo8UC4: function(session, stream, meta, callback) {

        var buffer = new Buffer(0);

        stream.on('data', function(chunk) {
            buffer = Buffer.concat([buffer, chunk]);
        });

        stream.on('end', function() {

            if (buffer.length < 10) {
                console.log('Error: not enough data (' + buffer.length + ' bytes)');
                return;
            }

            session.feedPictureBlob(buffer, meta.id, meta.width, meta.height, meta.focalLen.x, meta.focalLen.y, meta.channels, function(pose) {
                callback(pose);
            });


        });

    },

    lz4To8UC4: function(session, stream, meta, callback) {

       var buffer = new Buffer(0);

        stream.on('data', function(chunk) {
            buffer = Buffer.concat([buffer, chunk]);
        });

        var uncompressed = lz4.decode(buffer);

        stream.on('end', function() {
    
            if (buffer.length < 10) {
                console.log('Error: not enough data');
                return;
            }

            session.feedPictureBlob(new Uint8Array(uncompressed), meta.id, meta.width, meta.height, meta.focalLen.x, meta.focalLen.y, meta.channels, function(pose) {
                callback(pose);
            });

        });

    },

    pakoTo8UC4: function(session, stream, meta, callback) {
        
        var inflator = new pako.Inflate();

        stream.on('data', function(chunk) {
            inflator.push(chunk, false);
        });

        stream.on('end', function() {

            var result = inflator.result;

            session.feedPictureBlob(result, meta.id, meta.width, meta.height, meta.focalLen.x, meta.focalLen.y, meta.channels, function(pose) {
                callback(pose);
            });
        });
    },

    pakoToPNG: function(session, stream, meta, callback) {
        
        var inflator = new pako.Inflate();

        stream.on('data', function(chunk) {
            inflator.push(chunk, false);
        });

        stream.on('end', function() {

            var writeBuffer = new streamBuff.WritableStreamBuffer({
                initialSize: (100 * 1024),
                incrementAmount: (10 * 1024)
            });

            var result = inflator.result;

            if (result.length < 10) {
                console.log('not enought data');
                return;
            }
            
            var png = new PNG({
                width: meta.width,
                height: meta.height
            });
            
            png.data = new Buffer(result);
            
            png.pack().pipe(writeBuffer).once('close', function () {
                session.feedPictureFile(writeBuffer.getContents(), meta.id, meta.focalLen.x, meta.focalLen.y, meta.channels, function(pose) {
                    callback(pose);
                });
            });
        });
    },

    blobToFile: function(session, stream, meta, callback) {
        
        var buffer = new Buffer(0);

        stream.on('data', function(chunk) {
            buffer = Buffer.concat([buffer, chunk]);
        });

        stream.on('end', function() {
    
            if (buffer.length < 10) {
                console.log('Error: not enough data');
                process.exit();
            }

            session.feedPictureFile(buffer, meta.channels, function(pose) {
                callback(pose);
            });
        });
    }
    
}

