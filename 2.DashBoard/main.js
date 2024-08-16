var mycanvas = document.getElementById("mycanvas");
var ctx = mycanvas.getContext("2d");

const WIDTH  = 800;
const HEIGHT = 800;

const TIME_STEP = 0.1;
const G = 10;

var Ball = 
{
    x: 0 + 800 / 2,
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
}

var draw_main = function ()
{
    window.requestAnimationFrame(simulation);
}


var simulation = function () 
{
    // clear canvas
    ctx.clearRect(0, 0, mycanvas.width, mycanvas.height);

    Ball.vy += TIME_STEP * G;

    Ball.x += TIME_STEP * Ball.vx;
    Ball.y += TIME_STEP * Ball.vy;

    if (Ball.y > mycanvas.height || Ball.y < 0) 
    {
        Ball.vy = -Ball.vy;
    }
    if (Ball.x > mycanvas.width  || Ball.x < 0) 
    {
        Ball.vx = -Ball.vx;
    }

    Ball.draw();


   window.requestAnimationFrame(simulation);
}


init_main();
draw_main();