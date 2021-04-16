var dashboard = document.getElementById("dashboard");
var dash_ctx = dashboard.getContext("2d");

const CHART_WIDTH  = 400;
const CHART_HEIGHT = 400;

// data array
var coords_x = new Array(CHART_WIDTH);
var coords_y = new Array(CHART_WIDTH);
var energy_p = new Array(CHART_WIDTH); // Potential energy
var energy_k = new Array(CHART_WIDTH); // Kinetic energy
var energy   = new Array(CHART_WIDTH); // Mechanical energy = Potential + Kinetic

// states
var enableCoords_x = false;
var enableCoords_y = false;
var enableEnergy_p = false;
var enableEnergy_k = false;
var enableEnergy   = false; 

var onCoords_x = function ()
{
    enableCoords_x = !enableCoords_x;
}

var onCoords_y = function ()
{
    enableCoords_y = !enableCoords_y;
}

var onEnergy_p = function ()
{
    enableEnergy_p = !enableEnergy_p;
}

var onEnergy_k = function ()
{
    enableEnergy_k = !enableEnergy_k;
}

var onEnergy = function ()
{
    enableEnergy = !enableEnergy;
}


var init = function ()
{
    dashboard.width = CHART_WIDTH;
    dashboard.height = CHART_HEIGHT;

    // init data
    for (var i = 0; i < CHART_WIDTH; i++)
    {
        coords_x[i] = -100;
        coords_y[i] = -100;
        energy_p[i] = -100;
        energy_k[i] = -100;
        energy[i]   = -100;
    }

    window.requestAnimationFrame(draw);
}


var normalization = function (x, range)
{
    return x / range;
}


var update_data = function ()
{
    coords_x.pop();
    coords_x.unshift(Ball.x);

    coords_y.pop();
    coords_y.unshift(Ball.y);

    energy_p.pop();
    var curr_energy_p = Ball.mass * G * (HEIGHT - Ball.y);
    energy_p.unshift(curr_energy_p);

    energy_k.pop();
    var curr_energy_k = 0.5 * Ball.mass * (Ball.vx * Ball.vx + Ball.vy * Ball.vy);
    energy_k.unshift(curr_energy_k);

    energy.pop();
    energy.unshift(curr_energy_p + curr_energy_k);
}

var draw = function ()
{
    // clear canvas
    dash_ctx.clearRect(0, 0, dashboard.width, dashboard.height);

    update_data();

    if (enableCoords_x) 
    {
        for (var i = 0; i < CHART_WIDTH; i++)
        {
            var x = i;
            var y = CHART_WIDTH * normalization(coords_x[i], 1000);
            dash_ctx.fillStyle = 'blue';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableCoords_y)
    {
        for (var i = 0; i < CHART_WIDTH; i++)
        {
            var x = i;
            var y = CHART_WIDTH * normalization(coords_y[i], 800);
            dash_ctx.fillStyle = 'red';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableEnergy_p)
    {
        var max_energy = Ball.mass * G * HEIGHT;
        for (var i = 0; i < CHART_WIDTH; i++)
        {
            var x = i;
            var y = CHART_HEIGHT - CHART_HEIGHT * normalization(energy_p[i], max_energy);
            dash_ctx.fillStyle = 'green';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableEnergy_k)
    {
        var max_energy = Ball.mass * G * HEIGHT;
        for (var i = 0; i < CHART_WIDTH; i++)
        {
            var x = i;
            var y = CHART_HEIGHT - CHART_HEIGHT * normalization(energy_k[i], max_energy);
            dash_ctx.fillStyle = 'blue';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableEnergy) 
    {
        var max_energy = Ball.mass * G * HEIGHT;
        for (var i = 0; i < CHART_WIDTH; i++)
        {
            var x = i;
            var y = CHART_HEIGHT - CHART_HEIGHT * normalization(energy[i], max_energy);
            dash_ctx.fillStyle = 'red';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    
    
    window.requestAnimationFrame(draw);
}




init();
draw();