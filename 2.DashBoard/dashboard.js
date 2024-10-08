let dashboard = document.getElementById("dashboard");
let dash_ctx = dashboard.getContext("2d");

const CHART_WIDTH  = 400;
const CHART_HEIGHT = 400;

// data array
let coords_x = new Array(CHART_WIDTH);
let coords_y = new Array(CHART_WIDTH);
let energy_p = new Array(CHART_WIDTH); // Potential energy
let energy_k = new Array(CHART_WIDTH); // Kinetic energy
let energy   = new Array(CHART_WIDTH); // Mechanical energy = Potential + Kinetic

// states
let enableCoords_x = false;
let enableCoords_y = false;
let enableEnergy_p = false;
let enableEnergy_k = false;
let enableEnergy   = false; 

let onCoords_x = function ()
{
    enableCoords_x = !enableCoords_x;
}

let onCoords_y = function ()
{
    enableCoords_y = !enableCoords_y;
}

let onEnergy_p = function ()
{
    enableEnergy_p = !enableEnergy_p;
}

let onEnergy_k = function ()
{
    enableEnergy_k = !enableEnergy_k;
}

let onEnergy = function ()
{
    enableEnergy = !enableEnergy;
}


let init_dashboard = function ()
{
    dashboard.width = CHART_WIDTH;
    dashboard.height = CHART_HEIGHT;

    // init data
    for (let i = 0; i < CHART_WIDTH; i++)
    {
        coords_x[i] = -100;
        coords_y[i] = -100;
        energy_p[i] = -100;
        energy_k[i] = -100;
        energy[i]   = -100;
    }

    window.requestAnimationFrame(draw_dashboard);
}


let normalization = function (x, range)
{
    return x / range;
}


let update_data = function ()
{
    coords_x.pop();
    coords_x.unshift(Ball.x);

    coords_y.pop();
    coords_y.unshift(Ball.y);

    energy_p.pop();
    let curr_energy_p = Ball.mass * G * (HEIGHT - Ball.y);
    energy_p.unshift(curr_energy_p);

    energy_k.pop();
    let curr_energy_k = 0.5 * Ball.mass * (Ball.vx * Ball.vx + Ball.vy * Ball.vy);
    energy_k.unshift(curr_energy_k);

    energy.pop();
    energy.unshift(curr_energy_p + curr_energy_k);
}

let draw_dashboard = function ()
{
    // clear canvas
    dash_ctx.clearRect(0, 0, dashboard.width, dashboard.height);

    update_data();

    if (enableCoords_x) 
    {
        for (let i = 0; i < CHART_WIDTH; i++)
        {
            let x = i;
            let y = CHART_WIDTH * normalization(coords_x[i], 1000);
            dash_ctx.fillStyle = 'Red';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableCoords_y)
    {
        for (let i = 0; i < CHART_WIDTH; i++)
        {
            let x = i;
            let y = CHART_WIDTH * normalization(coords_y[i], 800);
            dash_ctx.fillStyle = 'green';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableEnergy_p)
    {
        let max_energy = Ball.mass * G * HEIGHT;
        for (let i = 0; i < CHART_WIDTH; i++)
        {
            let x = i;
            let y = CHART_HEIGHT - CHART_HEIGHT * normalization(energy_p[i], max_energy);
            dash_ctx.fillStyle = 'DodgerBlue';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableEnergy_k)
    {
        let max_energy = Ball.mass * G * HEIGHT;
        for (let i = 0; i < CHART_WIDTH; i++)
        {
            let x = i;
            let y = CHART_HEIGHT - CHART_HEIGHT * normalization(energy_k[i], max_energy);
            dash_ctx.fillStyle = 'Gold';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableEnergy) 
    {
        let max_energy = Ball.mass * G * HEIGHT;
        for (let i = 0; i < CHART_WIDTH; i++)
        {
            let x = i;
            let y = CHART_HEIGHT - CHART_HEIGHT * normalization(energy[i], max_energy);
            dash_ctx.fillStyle = 'SlateGray';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    
    
    window.requestAnimationFrame(draw_dashboard);
}




init_dashboard();
draw_dashboard();