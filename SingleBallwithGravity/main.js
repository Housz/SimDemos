var mycanvas = document.getElementById("mycanvas");
var ctx = mycanvas.getContext("2d");

const WIDTH  = 1500;
const HEIGHT = 800;

const TIME_STEP = 0.1;
const G = 10;

var ball = 
{
    x: 0,
    y: 0,
    vx: 10,
    vy: 10,
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



var init = function ()
{
    mycanvas.width = WIDTH;
    mycanvas.height = HEIGHT;
}

var draw = function ()
{
    window.requestAnimationFrame(simulation);
}


var clear_canvas = function ()
{
    ctx.clearRect(0, 0, mycanvas.width, mycanvas.height);
}


var simulation = function () 
{
    clear_canvas();

    ball.vy += TIME_STEP * G;

    ball.x += TIME_STEP * ball.vx;
    ball.y += TIME_STEP * ball.vy;

    if (ball.y > mycanvas.height || ball.y < 0) 
    {
        ball.vy = -ball.vy;
    }
    if (ball.x > mycanvas.width  || ball.x < 0) 
    {
        ball.vx = -ball.vx;
    }

    ball.draw();


   window.requestAnimationFrame(simulation);
}


init();
draw();