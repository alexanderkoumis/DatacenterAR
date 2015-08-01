function convert8UC4To8UC1(imageData8UC4) {

    var length8UC4 = imageData8UC4.length;
    var length8UC1 = length8UC4 / 4;
    var imageData8UC1 = new Uint8Array(length8UC1);

    var idx8UC4 = 0;
    for (var i = 0; i < length8UC1; ++i) {
        var r = imageData8UC4[idx8UC4];
        var g = imageData8UC4[idx8UC4+1];
        var b = imageData8UC4[idx8UC4+2];
        imageData8UC1[i] = Math.floor(0.2126 * r + 0.7152 * g + 0.0722 * b);
        idx8UC4 += 4;
    }

    return imageData8UC1;
}

onmessage = function(image8UC4) {

    var imageData8UC1 = convert8UC4To8UC1(image8UC4.data.data);
    postMessage(imageData8UC1);

}