var fs                 = require('fs');
var express            = require('express');
var app                = express();
var server             = require('http').Server(app);
var binaryjs           = require('binaryjs');
var streamBuff         = require('stream-buffers');

var roslink            = require('../ros_link/build/Release/ROSLink');
var streamPicInterface = require('./stream-pic-interface');
var avgFPS             = require('./avg-fps');

app.use(express.static(__dirname + '/../public'));

var ports = {
    http: 3000,
    binary: 9000,
}

var binaryServer = binaryjs.BinaryServer({port: ports.binary});
var picNum = 0;
var picsMax = 1000;

binaryServer.on('connection', function(client) {

    var session = new roslink.Session(function() {
        console.log('session initialized');
    });

    console.log('connected');

    client.binaryType = 'arraybuffer';

    var interfaces = {
        'rawTo8UC4': streamPicInterface.rawTo8UC4,
        'lz4To8UC4': streamPicInterface.lz4To8UC4,
        'pakoTo8UC4': streamPicInterface.pakoTo8UC4,
        'pakoToPNG': streamPicInterface.pakoToPNG,
        'blobToFile': streamPicInterface.blobToFile
    }

    var started = false;
    var startTime;

    client.on('stream', function(stream, meta) {

        if (!started) {
            started = true;
            startTime = new Date();
        }

        interfaces[meta.interface](session, stream, meta, function(pose) {

            client.send({}, {type: 'pose', data: pose});

            if (picNum++ == picsMax) {
                var endTime = new Date();
                var seconds = (endTime - startTime) / 1000;
                var hz = picsMax / seconds;
                console.log(seconds + ' seconds for ' + picsMax + ' frames (' + hz + ') hz');
                process.exit();
            }

            avgFPS.tick();
            avgFPS.print();

        });

    });

    client.on('close', function() {
        console.log('client connection closing...');
    });

    client.on('error', function(error) {
        console.log('error: ' + error);
        client.close();
    });

});

server.listen(ports.http);
