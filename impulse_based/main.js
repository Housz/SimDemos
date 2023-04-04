let  mycanvas = document.getElementById("mycanvas");
let  ctx = mycanvas.getContext("2d");

const WIDTH  = 800;
const HEIGHT = 800;

const TIME_STEP = 1 / 60.0;

const G = -10;
const PI = 3.14159265358979323846264;

class vec2
{
	constructor(x = 0.0, y = 0.0)
	{
		this.x = x;
		this.y = y;
	}

	set(v) 
	{
		this.x = v.x; this.y = v.y;
	}

	add(v)
	{
		this.x += v.x;
		this.y += v.y;
	}

	sub(v)
	{
		this.x -= v.x;
		this.y -= v.y;
	}

	scale(s)
	{
		this.x *= s;
		this.y *= s;
	}

	length()
	{
		return Math.sqrt(this.x * this.x + this.y * this.y);
	}

	dot(v)
	{
		return this.x*v.x + this.y*v.y;
	}
}

class Rect
{
	constructor(pos, )
}