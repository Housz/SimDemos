var fuckca = document.getElementById("fuckca");
var ctx = fuckca.getContext("2d");


// status
// 0: empty   ---> 1 if probability > p
// 1: tree    ---> 2 if sum(up, down, left, right) > 4
// 2: burning ---> 0

const EMPTY = 0;
const TREE = 1;
const BURNING = 2;

const WIDTH  = 500;
const HEIGHT = 500;
const LEN    = 100;
const GRID   = 5;

const P_INIT = 80;
const P_GROW = 10;
const P_BURN = 20;

const COLOR_0 = "#CCCCCC";
const COLOR_1 = "#336600";
const COLOR_2 = "#CC0000";


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
    fuckca.width  = WIDTH;
    fuckca.height = HEIGHT;

    for (let i = 0; i < LEN + 2; i++) {
        for (let j = 0; j < LEN + 2; j++) {
            // margin
            if (i === 0 || i === LEN + 1 || j === 0 || j === LEN + 1) {
                arr[i][j] = 1;
                continue;
            }

            let p_i = Math.floor(Math.random() * 100);
            let p_b = Math.floor(Math.random() * 1000000);

            if (p_i < P_INIT)
                arr[i][j] = TREE;
            else
                arr[i][j] = EMPTY;

            if (p_b < P_BURN)
                arr[i][j] = BURNING;
        }
    }
};


var ctrl = function () {
    for (let i = 1; i <= LEN; i++) {
        for (let j = 1; j <= LEN; j++) {
            if (arr[i][j] === EMPTY) {
                let p_g = Math.floor(Math.random() * 100);
                if (p_g < P_GROW)
                    arr_t[i][j] = TREE;
                else
                    arr_t[i][j] = EMPTY;
                continue;
            }

            if (arr[i][j] === TREE) {
                let p_b = Math.floor(Math.random() * 10000000);
                if(arr[i-1][j] + arr[i][j-1] + arr[i+1][j] + arr[i][j+1] > 4
                    || p_b < P_BURN)
                    arr_t[i][j] = BURNING;
                else
                    arr_t[i][j] = TREE;
                continue;
            }

            if (arr[i][j] === BURNING) {
                arr_t[i][j] = EMPTY;
                continue;
            }

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
            if (arr[i][j] === EMPTY) {
                ctx.fillStyle = COLOR_0;
                ctx.fillRect((i-1) * GRID, (j-1) * GRID, GRID, GRID);
            }

            if (arr[i][j] === TREE) {
                ctx.fillStyle = COLOR_1;
                ctx.fillRect((i-1) * GRID, (j-1) * GRID, GRID, GRID);
            }

            if (arr[i][j] === BURNING) {
                ctx.fillStyle = COLOR_2;
                ctx.fillRect((i-1) * GRID, (j-1) * GRID, GRID, GRID);
            }
        }
    }
};

// test


init_rand();
show();
setInterval(function() {
    ctrl();
    show();
}, 50);


