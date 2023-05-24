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

var pointIcon = L.icon({
    iconUrl: "static/point.svg",
    iconSize: [16, 16],
    iconAnchor: [8, 8],
    popupAnchor: [10, -35]
});

//array de markers
var markers = [];

var simulation_files = [];
var selected_sim_file = undefined;

setInterval(obuCall, 1000);

$('#start-button').click(    
    function () {
        if(selected_sim_file === undefined || selected_sim_file === "Select a simulation file") {
            alert("You need to select a simulation file!");
        }
       
        $.ajax({
            url: '/start',
            type: 'post',
            contentType: 'application/json',
            data: JSON.stringify({"sim_file": selected_sim_file }),
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

$("#sim-file-list").on('change', function(e){
    selected_sim_file = this.value;
    console.log(`Selected file: ${this.value}`);
});

$(document).ready(function () {
    const sim_file_list = document.getElementById("sim-file-list");
    sim_file_list.innerHTML = "<option selected>Select a simulation file</option>";

    simulation_files = [];

    $.ajax({
        url: '/list',
        type: 'get',
        contentType: 'application/json',
        data: {},
        success: function (response) {
            console.log(response)
            for ( file in response.sim_files) {
                sim_file_list.innerHTML += `<option value=${response.sim_files[file]}>${response.sim_files[file]}</option>`;
                simulation_files.push(response.sim_files[file]);
            }
        }
    });
})

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
                for (const key in response) {
                    markers[i++] = L.marker([response[key]["latitude"], response[key]["longitude"]], { icon: obuIcon }).addTo(map)
                        .bindTooltip(key, { permanent: false });

                    total_progress += response[key]["progress"] * (1 / drone_count);

                    for(const coll_key in response[key]["probable_collision_points"]) {
                        const coll_point = response[key]["probable_collision_points"][coll_key];
                        console.log(coll_point);
                        
                        markers[i++] = L.marker([coll_point["latitude"], coll_point["longitude"]], { icon: pointIcon }).addTo(map)
                        .bindTooltip(key+coll_key, { permanent: false });
                    }
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
                        <th scope="row">Altitude (m)</th>
                        <td>${parseFloat(response[key]["altitude"].toFixed(3))}</td>
                      </tr>
                      <tr>
                        <th scope="row">Latitude</th>
                        <td>${parseFloat(response[key]["latitude"].toFixed(6))}</td>
                      </tr>
                      <tr>
                        <th scope="row">Longitude</th>
                        <td>${parseFloat(response[key]["longitude"].toFixed(6))}</td>
                      </tr>
                      <tr>
                        <th scope="row">Horizontal Velocity (m/s)</th>
                        <td>${parseFloat(response[key]["horizontal_velocity"].toFixed(3))}</td>
                      </tr>
                      <tr>
                        <th scope="row">Heading (°)</th>
                        <td>${parseFloat((response[key]["heading"] * (180 / Math.PI)).toFixed(3))}</td>
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
