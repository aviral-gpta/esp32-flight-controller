var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
window.addEventListener('load', onload);

var configMenu = '1';

function onload(event) {
    initWebSocket();
}

function getValues(){
    websocket.send("getValues_c="+configMenu);
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {
    console.log('Connection opened');
    getValues();
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function changeConfigMenu(element) {
    document.getElementById("button"+configMenu).classList.remove("active");
    element.classList.add("active");
    configMenu = element.id.charAt(element.id.length-1);
    getValues();
}

function onStop(element) {
    websocket.send("stop");
}

function updateSlider(element, sliderNumber) {
    var sliderValue = document.getElementById(element.id).value;
    document.getElementById(element.id + "Value").innerHTML = sliderValue;
    console.log("update_c="+configMenu+"s="+sliderNumber+"v="+sliderValue.toString());
    websocket.send("update_c="+configMenu+"s="+sliderNumber+"v="+sliderValue.toString());
}

function onMessage(event) {
    console.log(event.data);
    var myObj = JSON.parse(event.data);
    var keys = Object.keys(myObj);

    for (var i = 0; i < keys.length; i++){
        var key = keys[i];
        document.getElementById(key + "Value").innerHTML = myObj[key];
        document.getElementById(key).value = myObj[key];
    }
}
