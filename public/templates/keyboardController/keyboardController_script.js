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
let motorPowerPublisher = undefined;  // 추가: motor power publisher

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
}else{
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
	motorPowerPublisher = new ROSLIB.Topic({ // 추가: motor power publisher 초기화
		ros: rosbridge.ros,
		name: 'sirbot1/motor_power',
		messageType: 'std_msgs/Bool',
		queue_size: 1
	});

	// Subscribe to the mapsaved topic
	let mapSavedSubscriber = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: 'sirbot1/mapsaved',
		messageType: 'std_msgs/Bool',
	});

	let alertActive = false; // 플래그 초기화

	mapSavedSubscriber.subscribe((message) => {
		if (message.data && !alertActive) {
			alert('지도를 저장하였습니다.');
			alertActive = true;
			setTimeout(() => {
				alertActive = false; // 일정 시간 후 플래그 리셋
			}, 1000); // 1초 후 다시 팝업이 가능하도록 설정 (필요에 따라 시간 조절)
		}
	});

	// 프로그램이 처음 실행될 때 motor power 토픽을 5초 동안 게시
	let intervalID = setInterval(() => {
		publishMotorPower(true);
	}, 1000); // 1초마다 게시

	// 5초 후 게시 중지
	setTimeout(() => {
		clearInterval(intervalID);
	}, 5000); // 5초 후 중지
}

function publishTwist(linearX, linearY, angularZ) {
	const twist = new ROSLIB.Message({
		linear: { x: linearX, y: linearY, z: 0 },
		angular: { x: 0, y: 0, z: angularZ },
	});
	cmdVelPublisher.publish(twist);
}

function publishSave(){
	const boolMessage = new ROSLIB.Message({
    	data: true // true 값을 포함한 메시지 생성
  	});

  	saveCommandPublisher.publish(boolMessage);
}

function publishMotorPower(state){
	const boolMessage = new ROSLIB.Message({
    	data: state // true 값을 포함한 메시지 생성
  	});

  	motorPowerPublisher.publish(boolMessage);
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

//preview for moving around

let preview_active = false;

/**
 * 로봇 컨트롤 초기화 함수
 */
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
}

function throttle(mainFunction, delay) {
	let timerFlag = null; // Variable to keep track of the timer
  
	// Returning a throttled version 
	return (...args) => {
	  if (timerFlag === null) { // If there is no timer currently running
		mainFunction(...args); // Execute the main function 
		timerFlag = setTimeout(() => { // Set a timer to clear the timerFlag after the specified delay
		  timerFlag = null; // Clear the timerFlag to allow the main function to be executed again
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

	if(settings['{uniqueID}'].holonomic_swap){
		publishTwist(_linearVel, _angularVel, 0);
	}
	else{
		publishTwist(_linearVel, 0, _angularVel);
	}
	
}

initRobotControl();

console.log("Keyboard controller Widget Loaded {uniqueID}")
