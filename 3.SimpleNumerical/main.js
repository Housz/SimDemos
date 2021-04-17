var mycanvas = document.getElementById("mycanvas");
var ctx = mycanvas.getContext("2d");

const WIDTH  = 800;
const HEIGHT = 800;

const TIME_STEP = 1 / 120.0;
const NUM_SUB_STEPS = 10;
const G = 9.8;


var ball1, ball2, ball3;


class Ball
{
    constructor(x, y, vx, vy, mass, radius, color)
    {
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.mass = mass;
        this.radius = radius;
        this.color = color;
    }

    draw() 
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

    ball1 = new Ball(100, 0, 0, 0, 10, 25, 'red');
    ball2 = new Ball(300, 0, 0, 0, 10, 25, 'green');
    ball3 = new Ball(500, 0, 0, 0, 10, 25, 'blue');
    
    window.requestAnimationFrame(draw_main);
}

var draw_main = function ()
{
    // clear canvas
    ctx.clearRect(0, 0, mycanvas.width, mycanvas.height);

    for (let i = 0; i < NUM_SUB_STEPS; i++)
    {
        simulation();
    }

    ball1.draw();
    window.requestAnimationFrame(draw_main);
}


var simulation = function () 
{
    ball1.vy += TIME_STEP * G;

    ball1.x += TIME_STEP * ball1.vx;
    ball1.y += TIME_STEP * ball1.vy;


    // simple collision detect
    if (ball1.y > mycanvas.height || ball1.y < 0) 
    {
        ball1.vy = -ball1.vy;
    }
    if (ball1.x > mycanvas.width  || ball1.x < 0) 
    {
        ball1.vx = -ball1.vx;
    }
}


init_main();
draw_main();