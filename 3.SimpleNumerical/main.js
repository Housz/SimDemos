var mycanvas = document.getElementById("mycanvas");
var ctx = mycanvas.getContext("2d");

const WIDTH  = 800;
const HEIGHT = 800;

const TIME_STEP = 1 / 60.0;
const NUM_SUB_STEPS = 10;
const G = 9.8;

var Ball = 
{
    x: 0,
    y: 0,
    vx: 0,
    vy: 0,
    mass: 10,
    radius: 25,
    color: 'blue',
    draw: function() 
    {
        ctx.beginPath();
        ctx.arc(this.x, this.y, this.radius, 0, Math.PI * 2, true);
        ctx.closePath();
        ctx.fillStyle = this.color;
        ctx.fill();
    }
}


var init_main = function ()
{
    mycanvas.width = WIDTH;
    mycanvas.height = HEIGHT;
    
    window.requestAnimationFrame(draw_main);
}

var draw_main = function ()
{
    // clear canvas
    ctx.clearRect(0, 0, mycanvas.width, mycanvas.height);

    for (var i = 0; i < NUM_SUB_STEPS; i++)
    {
        simulation();
    }

    Ball.draw();
    window.requestAnimationFrame(draw_main);
}


var simulation = function () 
{
    Ball.vy += TIME_STEP * G;

    Ball.x += TIME_STEP * Ball.vx;
    Ball.y += TIME_STEP * Ball.vy;


    // simple collision detect
    if (Ball.y > mycanvas.height || Ball.y < 0) 
    {
        Ball.vy = -Ball.vy;
    }
    if (Ball.x > mycanvas.width  || Ball.x < 0) 
    {
        Ball.vx = -Ball.vx;
    }
}


init_main();
draw_main();