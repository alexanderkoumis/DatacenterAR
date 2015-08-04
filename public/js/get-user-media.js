var videoSources = [];
var camSelection = 0;
var userMediaSet = false;

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

function reallyGetUserMedia(camSelect, videoElem) {

    navigator.getUserMedia = (navigator.getUserMedia ||
                              navigator.webkitGetUserMedia ||
                              navigator.mozGetUserMedia ||
                              navigator.msGetUserMedia);

    var videoId = videoSources[camSelect];

    var constraints = {
        video: { optional: [{ sourceId: videoId }] },
        audio: false
    };

    navigator.getUserMedia(constraints, function(stream) {

        videoElem.src = window.URL.createObjectURL(stream);
        videoElem.play();

    }, function(error) { console.log('error: ' + error); });

}

function getVideoId() {

    return videoSources[camSelection];
    
}