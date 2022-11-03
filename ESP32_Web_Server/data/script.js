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
    if (element.classList.contains("active")) {
        websocket.send("start_c="+configMenu);
    } else {
        websocket.send("stop_c="+configMenu);    
    }
}

function onReset(element) {
    websocket.send("res_c="+configMenu);    
}

function updateSlider(element, sliderNumber) {
    var sliderValue = document.getElementById(element.id).value;
    if (sliderNumber == '5')
        sliderValue = 1000 + (document.getElementById("sliderMaxThrust").value - 1000) * sliderValue * 0.01;
    document.getElementById(element.id + "Value").innerHTML = sliderValue;
    console.log("update_c="+configMenu+"s="+sliderNumber+"v="+sliderValue.toString());
    websocket.send("update_c="+configMenu+"s="+sliderNumber+"v="+sliderValue.toString());
}

function onMessage(event) {
    console.log(event.data);
    var jsonPacket = JSON.parse(event.data);
    var keys = Object.keys(jsonPacket);
    var i = 0;

    for (; i < 4; i++){
        var key = keys[i];
        document.getElementById(key + "Value").innerHTML = jsonPacket[key];
        document.getElementById(key).value = jsonPacket[key];
    }

    var key = keys[4];
    document.getElementById(key + "Value").innerHTML = jsonPacket[key];
    document.getElementById(key).value = 100 * (jsonPacket[key] - 1000) / (document.getElementById("sliderMaxThrust").value - 1000);
    
    stop_button = document.getElementById("buttonS");
    if (jsonPacket[keys[5]] == "0") {
        stop_button.classList.add("active");
        stop_button.innerHTML = "START";
    } else {
        stop_button.classList.remove("active");
        stop_button.innerHTML = "STOP";
    }
}
