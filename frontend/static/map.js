var map = L.map('map').setView([40.631491, -8.656481], 16);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
}).addTo(map);

var obuIcon = L.icon({
    iconUrl: "static/drone.png",
    iconSize: [32, 32],
    iconAnchor: [18, 39],
    popupAnchor: [10, -35]
});

//array de markers
var markers = [];

setInterval(obuCall, 1000);

$('#start-button').click(
    function () {
        $.ajax({
            url: '/start',
            type: 'post',
            contentType: 'application/json',
            data: {},
            success: function () {
                
            }
        })
    }
)

$('#stop-button').click(
    function () {
        $.ajax({
            url: '/stop',
            type: 'post',
            contentType: 'application/json',
            data: {},
            success: function () {
                
            }
        })
    }
)

function obuCall() {
    $(document).ready(function () {

        $.ajax({
            url: '/drone-data',
            type: 'get',
            contentType: 'application/json',
            data: {},
            success: function (response) {
                markers.forEach(delMarker);

                const drone_count = Object.keys(response).length;
                const simulation_status = document.getElementById("simulation-status");
                const simulation_progress = document.getElementById("simulation-progress");
                const raw_data = document.getElementById("raw-drone-data");

                if (drone_count === 0) {
                    simulation_status.innerHTML = "<i>The Simulation is not running.</i><br><br>";
                } else {
                    simulation_status.innerHTML = "<i>The Simulation is running...</i><br><br>";
                }

                let total_progress = 0;
                let i = 0;
                for (var key in response) {
                    markers[i] = L.marker([response[key]["latitude"], response[key]["longitude"]], { icon: obuIcon }).addTo(map)
                        .bindTooltip(key, { permanent: false });

                    total_progress += response[key]["progress"] * (1 / drone_count);
                    i++;
                }

                total_progress = Math.round(total_progress);

                simulation_progress.innerHTML = `<div class="progress-bar" style="width: ${total_progress}%" role="progressbar" aria-valuenow="100" aria-valuemin="0"
                        aria-valuemax="100">${total_progress}%</div>
                        </div><br>`;

                raw_data.innerHTML = "<h3>Drone Data: </h3>";
                for (var key in response) {
                    raw_data.innerHTML += `<table class="table">
                    <thead>
                      <tr>
                        <th scope="col" class="text-danger">Drone </th>
                        <td scope="col" class="text-danger">${response[key]["drone_id"]}</td>
                      </tr>
                    </thead>
                    <tbody>
        
                      <tr>
                        <th scope="row">Altitude</th>
                        <td>${response[key]["altitude"]}</td>
                      </tr>
                      <tr>
                        <th scope="row">Latitude</th>
                        <td>${response[key]["latitude"]}</td>
                      </tr>
                      <tr>
                        <th scope="row">Longitude</th>
                        <td>${response[key]["longitude"]}</td>
                      </tr>
                      <tr>
                        <th scope="row">Horizontal Velocity</th>
                        <td>${response[key]["horizontal_velocity"]}</td>
                      </tr>
                      <tr>
                        <th scope="row">Heading</th>
                        <td>${response[key]["heading"]}</td>
                      </tr>        
                    </tbody>
                  </table>`
                }

                if (drone_count === 0) {
                    raw_data.innerHTML += "<h6>No data available </h6>";
                }
            }
        })


    })
}

function delMarker(value, index, array) {
    map.removeLayer(value)
}
