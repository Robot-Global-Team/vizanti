let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let Status = StatusModule.Status;

const SCREEN_CAPTURE_TOPIC = "/screen_capture/compressed";
const MESSAGE_TYPE = "sensor_msgs/CompressedImage";
const THROTTLE_MS = 1000;

const image = document.getElementById("{uniqueID}_image");
const status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let image_topic = undefined;
let listener = undefined;

function isJpeg(message) {
	const format = String(message.format || "").toLowerCase();
	return format.includes("jpeg") || format.includes("jpg");
}

function connect() {
	if (image_topic !== undefined)
		image_topic.unsubscribe(listener);

	image.src = "assets/tile_loading.png";
	status.setWarn("No data received.");

	image_topic = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: SCREEN_CAPTURE_TOPIC,
		messageType: MESSAGE_TYPE,
		throttle_rate: THROTTLE_MS,
	});

	listener = image_topic.subscribe((message) => {
		if (!isJpeg(message)) {
			image.src = "assets/tile_error.png";
			status.setError("Expected a JPEG screen capture.");
			return;
		}

		const source = "data:image/jpeg;base64," + message.data;
		const decoded = new Image();
		decoded.onload = () => {
			image.src = source;
			status.setOK(SCREEN_CAPTURE_TOPIC);
		};
		decoded.onerror = () => {
			image.src = "assets/tile_error.png";
			status.setError("Invalid JPEG screen capture.");
		};
		decoded.src = source;
	});

}

window.addEventListener("remove_widget", (event) => {
	if (event.uniqueID === "{uniqueID}" && image_topic !== undefined)
		image_topic.unsubscribe(listener);
});

connect();

console.log("Screen Capture Widget Loaded {uniqueID}");
