var ros;
var upper_limit_sub;
var lower_limit_sub;
var vel_setpoint_sub;

var gui_up_pub;
var gui_down_pub;
var spd_setpoint_pub;

var set_zero_torque_srv;
var set_lower_limit_srv;
var set_upper_limit_srv;
var reset_lower_limit_srv;
var reset_upper_limit_srv;
var enable_lower_limit_srv;
var enable_upper_limit_srv;
var pwr_button_srv;
var setpoint_srv;

var upper_limit;
var lower_limit;
var speed_setpoint = 15;
var up_btn_cmd = false;
var down_btn_cmd = false;

var slider_busy = false;

var mainTimer;

var MAX_SPEED = 30.0;

var enable_loop = true;
function stopLoop() {
  enable_loop = false;
  $('#status_spinner_color').removeClass();
  $('#status_spinner_color').addClass('text-danger');
  $('#status_spinner').removeClass();
  $('#status_spinner').addClass('spinner-grow spinner-grow-sm');
}

function UpdatePage() {
  $('#upper_limit').val(upper_limit);
  $('#lower_limit').val(lower_limit);
  // $('#speed_slider').attr('value', speed_setpoint);
  // $('#speed_slider').val(speed_setpoint);
  // rendu a mettre a jour le slider en fonction du topic setpoint
}

function publishMessage() {
  /* publish up button state */
  var msg1 = new ROSLIB.Message({
    data: up_btn_cmd,
  });
  gui_up_pub.publish(msg1);

  /* publish down button state */
  var msg2 = new ROSLIB.Message({
    data: down_btn_cmd,
  });
  gui_down_pub.publish(msg2);
}


function mainLoop() {
  if (enable_loop) {
    // console.log(speed_setpoint);
    UpdatePage();
    publishMessage();
  }
}

function SetLowerLimit() {
  console.log('SetLowerLimit');

  var request = new ROSLIB.ServiceRequest({});
  set_lower_limit_srv.callService(request, function (result) { });
}

function SetUpperLimit() {
  console.log('SetUpperLimit');

  var request = new ROSLIB.ServiceRequest({});
  set_upper_limit_srv.callService(request, function (result) { });
}

function ResetLowerLimit() {
  console.log('ResetLowerLimit');

  var request = new ROSLIB.ServiceRequest({});
  reset_lower_limit_srv.callService(request, function (result) { });
}

function ResetUpperLimit() {
  console.log('ResetUpperLimit');

  var request = new ROSLIB.ServiceRequest({});
  reset_upper_limit_srv.callService(request, function (result) { });
}

function EnableLowerLimit(value) {
  console.log('EnableLowerLimit');

  var request = new ROSLIB.ServiceRequest({
    data: value
  });
  enable_lower_limit_srv.callService(request, function (result) { });
}

function EnableUpperLimit(value) {
  console.log('EnableUpperLimit');

  var request = new ROSLIB.ServiceRequest({
    data: value
  });
  enable_upper_limit_srv.callService(request, function (result) { });
}

function SetZeroTorque(value) {
  console.log('SetZeroTorque');

  var request = new ROSLIB.ServiceRequest({});
  // set_zero_torque_srv.callService(request, function (result) { });
}

function OnUpBtnPressed() {
  up_btn_cmd = true;
}

function OnDownBtnPressed() {
  down_btn_cmd = true;
}

function OnUpBtnRelease() {
  up_btn_cmd = false;
}

function OnDownBtnRelease() {
  down_btn_cmd = false;
}

function OnSetpointChanged(value) {
  console.log('OnSetpointChanged');

  var request = new ROSLIB.ServiceRequest({
    value: parseFloat(value)
  });
  setpoint_srv.callService(request, function (result) { });
}


window.onload = function () {
  up_btn_cmd = false;
  down_btn_cmd = false;

  var robot_IP = document.domain;
  if (robot_IP == "") {
    robot_IP = "localhost";
  }
  var ros_master_uri = "ws://" + robot_IP + ":9090";

  // Connecting to ROS
  // -----------------
  ros = new ROSLIB.Ros({
    url: ros_master_uri
  });

  ros.on('connection', function () {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
    stopLoop();
  });

  ros.on('close', function () {
    console.log('Connection to websocket server closed.');
    stopLoop();
  });

  // Setup topic subscribers
  // ----------------------
  upper_limit_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/armms_control/upper_limit',
    messageType: 'std_msgs/Float64'
  });
  upper_limit_sub.subscribe(function (message) {
    console.debug('Received message on ' + upper_limit_sub.name + ': ' + message.data);
    upper_limit = message.data;
  });

  lower_limit_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/armms_control/lower_limit',
    messageType: 'std_msgs/Float64'
  });
  lower_limit_sub.subscribe(function (message) {
    console.debug('Received message on ' + lower_limit_sub.name + ': ' + message.data);
    lower_limit = message.data;
  });

  vel_setpoint_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/armms_control/velocity_setpoint',
    messageType: 'std_msgs/Float64'
  });
  vel_setpoint_sub.subscribe(function (message) {
    console.debug('Received message on ' + vel_setpoint_sub.name + ': ' + message.data);
    if(slider_busy == false)
    {
      speed_setpoint = message.data;
      $('#speed_slider').attr('value', speed_setpoint);
      $('#speed_slider').val(speed_setpoint);
      $('#speed_slider').next().html(speed_setpoint);
    }
  });

  // Get ros parameters
  // ----------------------
  ros.getParams(function(params) {
    console.log(params);
  });
  var vel_param = new ROSLIB.Param({
    ros : ros,
    name : '/joint_limits/joint1/max_velocity'
  });
  vel_param.get(function(value) {
    console.log('MAX VAL: ' + value);
    MAX_SPEED = value;
  });
  // Setup topic publishers 
  // ----------------------
  gui_up_pub = new ROSLIB.Topic({
    ros: ros,
    name: '/armms_webgui/user_button_up',
    messageType: 'std_msgs/Float64'
  });

  gui_down_pub = new ROSLIB.Topic({
    ros: ros,
    name: '/armms_webgui/user_button_down',
    messageType: 'std_msgs/Float64'
  });

  // Setup service 
  // ----------------------
  set_lower_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_control/set_lower_limit',
    serviceType: 'std_srvs/Empty'
  });

  set_upper_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_control/set_upper_limit',
    serviceType: 'std_srvs/Empty'
  });

  reset_lower_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_control/reset_lower_limit',
    serviceType: 'std_srvs/Empty'
  });

  reset_upper_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_control/reset_upper_limit',
    serviceType: 'std_srvs/Empty'
  });

  enable_lower_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_control/enable_lower_limit',
    serviceType: 'std_srvs/SetBool'
  });

  enable_upper_limit_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_control/enable_upper_limit',
    serviceType: 'std_srvs/SetBool'
  });

  pwr_button_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_rpi/power_button_event',
    serviceType: 'armms_msgs/ButtonEvent'
  });

  setpoint_srv = new ROSLIB.Service({
    ros: ros,
    name: '/armms_control/set_velocity_setpoint',
    serviceType: 'armms_msgs/SetVelocitySetpoint'
  });


  $('#speed_slider').attr('max', MAX_SPEED);
  $('#speed_slider').attr('min', 0.0);
  $('#speed_slider').attr('value', speed_setpoint);
  $('#speed_slider').val(speed_setpoint);

  rangeSlider();



  $('#enable_upperlim')[0].checked = true;
  $('#enable_lowerlim')[0].checked = true;
  EnableUpperLimit(true);
  EnableLowerLimit(true);

  // Setup main loop timer
  // ----------------------
  mainTimer = setInterval(function () { mainLoop(); }, 100);
}

var rangeSlider = function () {
  var slider = $('.range-slider'),
    range = $('.range-slider__range'),
    value = $('.range-slider__value');

    slider.each(function () {
      value.each(function () {
        var value = $(this).prev().attr('value');
        $(this).html(value);
      });

      range.on('input', function () {
        $(this).next().html(this.value);
      });
  });
};

$(function () {

  $('#set_lowerlim')
    .click(function () {
      SetLowerLimit();
    });

  $('#set_upperlim')
    .click(function () {
      SetUpperLimit();
    });

  $('#reset_lowerlim')
    .click(function () {
      ResetLowerLimit();
    });

  $('#reset_upperlim')
    .click(function () {
      ResetUpperLimit();
    });

  $('#up_btn')
    .mousedown(function () {
      OnUpBtnPressed();
    })
    .mouseup(function () {
      OnUpBtnRelease();
    });

  $('#down_btn')
    .mousedown(function () {
      OnDownBtnPressed();
    })
    .mouseup(function () {
      OnDownBtnRelease();
    });

  $('#set_zero_torque')
    .click(function () {
      SetZeroTorque();
    });

  $('#enable_lowerlim')
    .change(function () {
      var state = $('#enable_lowerlim')[0].checked;
      EnableLowerLimit(state);
    });

  $('#enable_upperlim')
    .change(function () {
      var state = $('#enable_upperlim')[0].checked;
      EnableUpperLimit(state);
    });

  /* Configure speed slider calback */
  $('#speed_slider')
  .on('change', function () {
    OnSetpointChanged(this.value);
    slider_busy = false;
  })
  .on('input', function () {
    slider_busy = true;
  });

});


