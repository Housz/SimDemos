var mycanvas = document.getElementById("mycanvas");
var ctx = mycanvas.getContext("2d");

const WIDTH  = 800;
const HEIGHT = 800;

const TIME_STEP = 1 / 60.0;
const NUM_SUB_STEPS = 1;
const G = 1;

var curr_time;


var ball0, ball1, ball2, ball3;


class Ball
{
    constructor(x, y, vx, vy, mass, radius, color, integrator_type)
    {
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.mass = mass;
        this.radius = radius;
        this.color = color;
        this.integrator_type = integrator_type;
    }

    draw() 
    {
        ctx.beginPath();
        ctx.arc(this.x, this.y, this.radius, 0, Math.PI * 2, true);
        ctx.closePath();
        ctx.fillStyle = this.color;
        ctx.fill();
    }

    update()
    {
        switch (this.integrator_type) 
        {
            case 0: // closed form
                
                let real_time = curr_time * 1 / 60;
                this.y = 0.5 * G * real_time * real_time + 0 * 0 + 0;

                break;

            case 1: 
                
                this.y  += TIME_STEP * this.vy;
                this.vy += TIME_STEP * G;
                
                break;
            
            case 2: 

                this.vy += TIME_STEP * G;
                this.y  += TIME_STEP * this.vy;
                
                break;

            case 3:

                let pre_v = this.vy;
                this.vy += TIME_STEP * G;
                this.y  += TIME_STEP * ( this.vy + pre_v ) / 2.0 ;
                
                break;

        }

        // naive collision detection
        // if ( this.y > mycanvas.height ||  this.y < 0) 
        // {
        //      this.vy = - this.vy;
        // }
        // if ( this.x > mycanvas.width  ||  this.x < 0) 
        // {
        //      this.vx = - this.vx;
        // }

    }

}




var init_main = function ()
{
    mycanvas.width = WIDTH;
    mycanvas.height = HEIGHT;

    curr_time = 0;

    ball0 = new Ball(100, 0, 0, 0, 10, 10, 'teal', 0);
    ball1 = new Ball(300, 0, 0, 0, 10, 10, 'red', 1);
    ball2 = new Ball(500, 0, 0, 0, 10, 10, 'green', 2);
    ball3 = new Ball(700, 0, 0, 0, 10, 10, 'blue', 3);
    
    window.requestAnimationFrame(draw_main);
}

var draw_main = function ()
{
    // clear canvas
    ctx.clearRect(0, 0, mycanvas.width, mycanvas.height);

    for (let i = 0; i < NUM_SUB_STEPS; i++)
    {
        simulation();

        curr_time ++;
    }

    ball0.draw();
    ball1.draw();
    ball2.draw();
    ball3.draw();

    window.requestAnimationFrame(draw_main);
}


var simulation = function () 
{
    // ball1.vy += TIME_STEP * G;

    // ball1.x += TIME_STEP * ball1.vx;
    // ball1.y += TIME_STEP * ball1.vy;


    // // simple collision detect
    // if (ball1.y > mycanvas.height || ball1.y < 0) 
    // {
    //     ball1.vy = -ball1.vy;
    // }
    // if (ball1.x > mycanvas.width  || ball1.x < 0) 
    // {
    //     ball1.vx = -ball1.vx;
    // }
    ball0.update();
    ball1.update();
    ball2.update();
    ball3.update();
}


init_main();
draw_main();