var dashboard = document.getElementById("dashboard");
var dash_ctx = dashboard.getContext("2d");


// states
var enableCoords_x = false;
var enableCoords_y = false;

var onCoords_x = function ()
{
    enableCoords_x = !enableCoords_x;
}

var onCoords_y = function ()
{
    enableCoords_y = !enableCoords_y;
}



var coords_x = new Array(400);
var coords_y = new Array(400);


var init = function ()
{
    dashboard.width = 400;
    dashboard.height = 400;

    for (var i = 0; i < 400; i++)
    {
        coords_x[i] = -10;
        coords_y[i] = -10;
    }

    window.requestAnimationFrame(draw);
}


var normalization = function (x, range)
{
    return x / range;
}

var update_dash = function ()
{
    coords_x.pop();
    coords_x.unshift(Ball.x);

    coords_y.pop();
    coords_y.unshift(Ball.y);
}

var draw = function ()
{
    dash_ctx.clearRect(0, 0, dashboard.width, dashboard.height);

    update_dash();

    if (enableCoords_x) 
    {
        for (var i = 0; i < 400; i++)
        {
            var x = i;
            var y = 400 * normalization(coords_x[i], 1000);
            dash_ctx.fillStyle = 'blue';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }

    if (enableCoords_y)
    {
        for (var i = 0; i < 400; i++)
        {
            var x = i;
            var y = 400 * normalization(coords_y[i], 800);
            dash_ctx.fillStyle = 'red';
            dash_ctx.fillRect(x, y, 2, 2);
        }
    }


    
    
    window.requestAnimationFrame(draw);
}




init();
draw();