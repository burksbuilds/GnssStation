#pragma once
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>GPS Station</title>
<script>
    var hostname = '';

function updateGnssData()
{
	fetch(hostname+'status')
	.then(function(response){
		return response.json();
	})
	.then(function(data){
		writeDataToPage('gnss_',data);
	})
	.catch(function (err){
		console.log('error updating Gnss data: '+err);
	});
}

function updateSystemData() {
    fetch(hostname + 'system')
        .then(function (response) {
            return response.json();
        })
        .then(function (data) {
            writeDataToPage('system_', data);
        })
        .catch(function (err) {
            console.log('error updating system data: ' + err);
        });
    }

function updatePositionData() {
    fetch(hostname + 'staticposition')
        .then(function (response) {
            return response.json();
        })
        .then(function (data) {
            writeDataToPage('position_', data);
        })
        .catch(function (err) {
            console.log('error updating position data: ' + err);
        });
    }

    function updateCasterData() {
        fetch(hostname + 'caster')
            .then(function (response) {
                return response.json();
            })
            .then(function (data) {
                writeDataToPage('caster_', data);
            })
            .catch(function (err) {
                console.log('error updating caster data: ' + err);
            });
    }

function writeDataToPage(prefix, data)
{
    console.log('writing ' + prefix + ' data to page: %o', data);
    console.log('keys: %o', Object.keys(data));
	for (var i=0; i < Object.keys(data).length; i++){
        var elementId = prefix + Object.keys(data)[i];
        console.log('evaluating ' + elementId + ' as ' + String(typeof(data[Object.keys(data)[i]])));
        if (typeof (data[Object.keys(data)[i]]) === 'object') {
            writeDataToPage(elementId + '_', data[Object.keys(data)[i]]);
        } else {
            var element = document.getElementById(elementId.toLowerCase());
            console.log('tagname='+element.tagName);
            if (element) {
                if (element.tagName === 'INPUT') {
                    console.log('type=' + element.getAttribute('type'));
                    if (element.getAttribute('type') === 'checkbox') {
                        element.checked = String(data[Object.keys(data)[i]]) === 'true';
                    } else {
                        element.value = String(data[Object.keys(data)[i]]);
                    }
                } else {
                    element.innerHTML = String(data[Object.keys(data)[i]]);
                }
            } else {
                console.log('no element found for data: ' + elementId);
            }
        }
	}
    }

    function restart() {
        const options = {
            method: 'POST',
            body: ''
        };

        fetch(hostname + 'restart',options);
    }

    function setPosition() {
        const newData =
        {
            X: parseFloat(getInputValueAndClear('position_x')),
            Y: parseFloat(getInputValueAndClear('position_y')),
            Z: parseFloat(getInputValueAndClear('position_z')),
            Acc: parseFloat(getInputValueAndClear('position_acc'))
        }

        const options = {
            method: 'POST',
            body: JSON.stringify(newData)
        };

        fetch(hostname + 'staticposition', options)
            .then(function (response) {
                return response.json();
            })
            .then(function (data) {
                writeDataToPage('position_', data);
            })
            .catch(function (err) {
                console.log('error updating position data: ' + err);
            });
    }

    function setCaster() {
        const newData =
        {
            Host: getInputValueAndClear('caster_host'),
            Port:parseInt(getInputValueAndClear('caster_port')),
            MountPoint: getInputValueAndClear('caster_mountpoint'),
            Password: getInputValueAndClear('caster_password'),
            Server: getInputValueAndClear('caster_server'),
            Enabled: getInputValueAndClear('caster_enabled')
        }

        const options = {
            method: 'POST',
            body: JSON.stringify(newData)
        };

        fetch(hostname + 'caster', options)
            .then(function (response) {
                return response.json();
            })
            .then(function (data) {
                writeDataToPage('caster_', data);
            })
            .catch(function (err) {
                console.log('error updating caster data: ' + err);
            });
    }

    function getInputValueAndClear(id) {
        var element = document.getElementById(id);
        var val;

        if (element.tagName === 'INPUT') {
            if (element.getAttribute('type') == 'checkbox') {
                val = element.checked;
                element.setAttribute('checked', false);
            } else {
                val = element.value;
                element.value = "";
            }
        }
        
        
        return val;
    }

    function startSurvey() {
        window.open(hostname + "rawdata.ubx?sampleduration=" + getInputValueAndClear("survey_time"), "_blank");
    }
</script>
</head>
<body>
    <h1>GNSS <button onclick="updateGnssData();">Refresh</button></h1>

    <h3>LLHA</h3>
    <table>
        <tr><td>Latitude (deg):</td><td id="gnss_llha_latitude"></td></tr>
        <tr><td>Longitude (deg):</td><td id="gnss_llha_longitude"></td></tr>
        <tr><td>Height above Ellipse (m):</td><td id="gnss_llha_height"></td></tr>
        <tr><td>Altitude above Sea (m):</td><td id="gnss_llha_altitude"></td></tr>
        <tr><td>Horizontal Accuracy (m):</td><td id="gnss_llha_horizontalaccuracy"></td></tr>
        <tr><td>Vertical Accuracy (m):</td><td id="gnss_llha_verticalaccuracy"></td></tr>
    </table>
    <h3>ECEF</h3>
    <table>
        <tr><td>X (m):</td><td id="gnss_ecef_x"></td></tr>
        <tr><td>Y (m):</td><td id="gnss_ecef_y"></td></tr>
        <tr><td>Z (m):</td><td id="gnss_ecef_z"></td></tr>
        <tr><td>3D Accuracy (m):</td><td id="gnss_ecef_3daccuracy"></td></tr>
    </table>
    <h3>Reception</h3>
    <table>
        <tr><td>Satellites in view:</td><td id="gnss_reception_satellitesinview"></td></tr>
        <tr><td>Fix Type:</td><td id="gnss_reception_fixtype"></td></tr>
        <tr><td>Carrier:</td><td id="gnss_reception_carrier"></td></tr>
    </table>
    <h1>System <button onclick="updateSystemData();">Refresh</button><button onclick="restart();">Restart</button></h1>
    <table>
        <tr><td>Signal Strength (dB):</td><td id="system_rssi"></td></tr>
        <tr><td>IP Address:</td><td id="system_ip"></td></tr>
        <tr><td>MAC Address:</td><td id="system_mac"></td></tr>
        <tr><td>Uptime (s):</td><td id="system_uptime"></td></tr>
    </table>
    <h1>RTK</h1>
    <h3>Fixed Position <button onclick="updatePositionData();">Refresh</button><button onclick="setPosition();">Write</button></h3>
    <table>
        <tr><td>X (m):</td><td><input value="" id="position_x"></td></tr>
        <tr><td>Y (m):</td><td><input value="" id="position_y"></td></tr>
        <tr><td>Z (m):</td><td><input value="" id="position_z"></td></tr>
        <tr><td>Accuracy (m):</td><td><input value="" id="position_acc"></td></tr>
    </table>

    <h3>NTRIP Caster <button onclick="updateCasterData();">Refresh</button><button onclick="setCaster();">Write</button></h3>
    <table>
        <tr><td>Host (and protocol):</td><td><input value="" id="caster_host"></td></tr>
        <tr><td>Port:</td><td><input value="" id="caster_port"></td></tr>
        <tr><td>Mount Point:</td><td><input value="" id="caster_mountpoint"></td></tr>
        <tr><td>Mount Point Password:</td><td><input value="" id="caster_password"></td></tr>
        <tr><td>Server Name:</td><td><input value="" id="caster_server"></td></tr>
        <tr><td>Transmition Enabled:</td><td><input type="checkbox" id="caster_enabled"></td></tr>
    </table>

    <h3>Survey Data</h3>
    <table>
        <tr><td>Survey Time (s)</td><td><input value="3600" id="survey_time" /></td><td><button onclick="startSurvey();">Start</button></td></tr>
    </table>
    <script>
        updateGnssData();
        updateSystemData();
        updatePositionData();
        updateCasterData();
    </script>
</body>
</html>
)rawliteral";