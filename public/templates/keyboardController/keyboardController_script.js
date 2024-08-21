let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

let topic = getTopic("{uniqueID}");
let status = new Status(
    document.getElementById("{uniqueID}_icon"),
    document.getElementById("{uniqueID}_status")
);

let cmdVelPublisher = undefined;
let saveCommandPublisher = undefined;
let motorPowerPublisher = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

// Sliders and checkboxes
const linearVelSlider = document.getElementById('{uniqueID}_linear_velocity');
const angularVelSlider = document.getElementById('{uniqueID}_angular_velocity');
const linearVelValue = document.getElementById('{uniqueID}_linear_velocity_value');
const angularVelValue = document.getElementById('{uniqueID}_angular_velocity_value');

linearVelSlider.addEventListener('input', function () {
    linearVelValue.textContent = this.value;
    saveSettings();
});

angularVelSlider.addEventListener('input', function () {
    angularVelValue.textContent = this.value;
    saveSettings();
});

linearVelSlider.addEventListener('change', saveSettings);
angularVelSlider.addEventListener('change', saveSettings);

// Settings
if (settings.hasOwnProperty('{uniqueID}')) {
    const loaded_data = settings['{uniqueID}'];
    topic = loaded_data.topic;

    linearVelSlider.value = loaded_data.linear_velocity;
    angularVelSlider.value = loaded_data.angular_velocity;

    linearVelValue.textContent = linearVelSlider.value;
    angularVelValue.textContent = angularVelSlider.value;
} else {
    saveSettings();
}

if(topic == ""){
    topic = "/cmd_vel";
    status.setWarn("No topic found, defaulting to /cmd_vel");
    saveSettings();
}

function saveSettings() {
    settings['{uniqueID}'] = {
        topic: topic,
        linear_velocity: parseFloat(linearVelSlider.value),
        angular_velocity: parseFloat(angularVelSlider.value),
    };
    settings.save();
}

// Topic and connections
async function loadTopics(){
    let result = await rosbridge.get_topics("geometry_msgs/Twist");
    let topiclist = "";
    result.forEach(element => {
        topiclist += "<option value='"+element+"'>"+element+"</option>"
    });
    selectionbox.innerHTML = topiclist;

    if(topic == "")
        topic = selectionbox.value;
    else{
        if(result.includes(topic)){
            selectionbox.value = topic;
        }else{
            topiclist += "<option value='"+topic+"'>"+topic+"</option>"
            selectionbox.innerHTML = topiclist
            selectionbox.value = topic;
        }
    }
    connect();
}

function connect(){
    cmdVelPublisher = new ROSLIB.Topic({
        ros: rosbridge.ros,
        name: topic,
        messageType: 'geometry_msgs/Twist',
        queue_size: 1
    });
    saveCommandPublisher = new ROSLIB.Topic({
        ros: rosbridge.ros,
        name: 'sirbot1/mapsaver',
        messageType: 'std_msgs/Bool',
        queue_size: 1
    });
    motorPowerPublisher = new ROSLIB.Topic({
        ros: rosbridge.ros,
        name: 'sirbot1/motor_power',
        messageType: 'std_msgs/Bool',
        queue_size: 1
    });

    let mapSavedSubscriber = new ROSLIB.Topic({
        ros: rosbridge.ros,
        name: 'sirbot1/mapsaved',
        messageType: 'std_msgs/Bool',
    });

    let alertActive = false;

    mapSavedSubscriber.subscribe((message) => {
        if (message.data && !alertActive) {
            alert('지도를 저장하였습니다.');
            alertActive = true;
            setTimeout(() => {
                alertActive = false;
            }, 1000);
        }
        else {
            alert('주행거리가 짧아 지도 저장 기능이 활성화되지 않았습니다.');
            alertActive = true;
            setTimeout(() => {
                alertActive = false;
            }, 1000);
        }
    });

    // Subscribe to the mapgen topic to display green circle
    let mapgenSubscriber = new ROSLIB.Topic({
        ros: rosbridge.ros,
        name: 'mapgen',
        messageType: 'std_msgs/Bool',
    });

    mapgenSubscriber.subscribe((message) => {
        if (message.data) {
            drawCircle('green');
        }
    });

    let intervalID = setInterval(() => {
        publishMotorPower(true);
    }, 1000);

    setTimeout(() => {
        clearInterval(intervalID);
    }, 5000);
    // 초기에는 빨간색 원을 그립니다.
    drawCircle('red');
}

function publishTwist(linearX, linearY, angularZ) {
    const twist = new ROSLIB.Message({
        linear: { x: linearX, y: linearY, z: 0 },
        angular: { x: 0, y: 0, z: angularZ },
    });
    cmdVelPublisher.publish(twist);
}

function publishSave() {
    const boolMessage = new ROSLIB.Message({
        data: true
    });
    saveCommandPublisher.publish(boolMessage);
}

function publishMotorPower(state){
    const boolMessage = new ROSLIB.Message({
        data: state
    });

    motorPowerPublisher.publish(boolMessage);
}

function drawCircle(color) {
    const canvas = document.getElementById('circleCanvas');
    const ctx = canvas.getContext('2d');

    ctx.clearRect(0, 0, canvas.width, canvas.height); // Clear the canvas

    ctx.beginPath();
    ctx.arc(canvas.width / 2, canvas.height / 2, 50, 0, 2 * Math.PI, false); // Draw a circle
    ctx.fillStyle = color;
    ctx.fill();
    ctx.lineWidth = 5;
    ctx.strokeStyle = '#003300';
    ctx.stroke();
}

selectionbox.addEventListener("change", (event) => {
    topic = selectionbox.value;
    saveSettings();
    connect();
    status.setOK();
});

selectionbox.addEventListener("click", (event) => {
    connect();
});

icon.addEventListener("click", (event) => {
    loadTopics();
});

loadTopics();

// 20Hz publishing variables and functions
let currentLinearVel = 0;
let currentAngularVel = 0;

function updateVelocities(linearX, angularZ) {
    currentLinearVel = linearX;
    currentAngularVel = angularZ;
}

function publishVelocities() {
    if(settings['{uniqueID}'].holonomic_swap){
        publishTwist(currentLinearVel, currentAngularVel, 0);
    }
    else{
        publishTwist(currentLinearVel, 0, currentAngularVel);
    }
}

setInterval(publishVelocities, 1000 / 20);  // 20Hz

function initRobotControl() {
    if (!window.RGT_CONFIG) {
        window.RGT_CONFIG = {};
    }

    if (window.RGT_CONFIG.KEYBOARD_EVENT_INITIALZIED) {
        return;
    }

    window.RGT_CONFIG.KEYBOARD_EVENT_INITIALZIED = true;

    const throttledEventHandler = throttle(robotControlEventHandler, 200);
    document.addEventListener('keydown', throttledEventHandler);
    document.addEventListener('keyup', function(event) {
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
            updateVelocities(0, 0);
        }
    });
}

function throttle(mainFunction, delay) {
    let timerFlag = null;
    return (...args) => {
        if (timerFlag === null) {
            mainFunction(...args);
            timerFlag = setTimeout(() => {
                timerFlag = null;
            }, delay);
        }
    };
}

/**
 * robotControlEventHandler
 * 
 * 키보드 입력을 받아서 조이스틱으로 방향에 대한 벡터 값을 보냅니다.
 * 
 * @param {KeyboardEvent} event 
 * @returns void
 */
function robotControlEventHandler(event) {
    const KEY = {
        LEFT: 'ArrowLeft',
        RIGHT: 'ArrowRight',
        UP: 'ArrowUp',
        DOWN: 'ArrowDown',
        SAVE: 's',
    }

    const DIRECTION = {
        [KEY.UP]: 'up',
        [KEY.RIGHT]: 'right',
        [KEY.DOWN]: 'down',
        [KEY.LEFT]: 'left',
        [KEY.SAVE]: 'save',
    }
    const direction = DIRECTION[event.key];

    if (!direction) {
        return;
    }

    let _linearVel = 0;
    let _angularVel = 0;
    const ANGULAR_WEIGHT = parseFloat(angularVelSlider.value);
    const LINEAR_WEIGHT = parseFloat(linearVelSlider.value);

    switch(direction) {
        case DIRECTION[KEY.UP]:
            _linearVel += LINEAR_WEIGHT;
            break;
        case DIRECTION[KEY.DOWN]:
            _linearVel -= LINEAR_WEIGHT;
            break;
        case DIRECTION[KEY.RIGHT]:
            _angularVel -= ANGULAR_WEIGHT;
            break;
        case DIRECTION[KEY.LEFT]:
            _angularVel += ANGULAR_WEIGHT;
            break;
        case DIRECTION[KEY.SAVE]:
            publishSave();
            break;
        default:
            break;
    }

    updateVelocities(_linearVel, _angularVel);
}

initRobotControl();

console.log("Keyboard controller Widget Loaded {uniqueID}")
