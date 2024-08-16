let mycanvas = document.getElementById("mycanvas");
let ctx = mycanvas.getContext("2d");


let init_main = function ()
{
    mycanvas.width = WIDTH;
    mycanvas.height = HEIGHT;
}

let draw_main = function ()
{
    window.requestAnimationFrame(simulation);
}


let simulation = function () 
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