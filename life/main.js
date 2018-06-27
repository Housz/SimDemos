var fuckca = document.getElementById("fuckca");
var ctx = fuckca.getContext("2d");


// status
// 0: empty   ---> 1 if probability > p
// 1: tree    ---> 2 if sum(up, down, left, right) > 4
// 2: burning ---> 0

const DEAD = 0;
const LIVE = 1;

const WIDTH = 800;
const HEIGHT = 800;
const LEN = 200;
const GRID = 4;

const P = 50;

const COLOR_1 = "#336600";


var arr = new Array(LEN + 2);
for (var i = 0; i < LEN + 2; i++) {
    arr[i] = new Array(LEN + 2);
}

var arr_t = new Array(LEN + 2);
for (var i = 0; i < LEN + 2; i++) {
    arr_t[i] = new Array(LEN + 2);
}
// ********
// *000000*
// *000000*
// *000000*
// *000000*
// *000000*
// *000000*
// ********


// controller
var init_rand = function () {
    fuckca.width = WIDTH;
    fuckca.height = HEIGHT;

    for (let i = 0; i < LEN + 2; i++) {
        for (let j = 0; j < LEN + 2; j++) {
            // margin
            if (i === 0 || i === LEN + 1 || j === 0 || j === LEN + 1) {
                arr[i][j] = 1;
                continue;
            }
            let p = Math.floor(Math.random() * 100);

            if (p < P)
                arr[i][j] = LIVE;
            else
                arr[i][j] = DEAD;
        }
    }
};


var ctrl = function () {
    for (let i = 1; i <= LEN; i++) {
        for (let j = 1; j <= LEN; j++) {

            let flag = arr[i - 1][j] + arr[i][j - 1] + arr[i + 1][j] + arr[i][j + 1]
                + arr[i - 1][j - 1] + arr[i + 1][j - 1] + arr[i + 1][j + 1] + arr[i - 1][j + 1];

            if (flag === 3)         // 3 live
                arr_t[i][j] = 1;
            else if (flag === 2)    // 2 stay
                continue;
            else
                arr_t[i][j] = 0;    // otherwise dead
            continue;

        }
    }

    for (let i = 1; i <= LEN; i++) {
        for (let j = 1; j <= LEN; j++) {
            arr[i][j] = arr_t[i][j];
        }
    }
};


// view
var show = function () {
    ctx.clearRect(0, 0, fuckca.width, fuckca.height);

    for (let i = 1; i <= LEN; i++) {
        for (let j = 1; j <= LEN; j++) {
            if (arr[i][j] === 1) {
                ctx.fillStyle = COLOR_1;
                ctx.fillRect((i - 1) * GRID, (j - 1) * GRID, GRID, GRID);
            }
        }
    }
};

// test


init_rand();
show();
setInterval(function () {
    ctrl();
    show();
}, 50);


