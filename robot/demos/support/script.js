import { readFromText } from '../../main.js';

// document.getElementById('fileInput').addEventListener('change', loadFile);

// function loadFile(event) {
//     const file = event.target.files[0];
//     const reader = new FileReader();
//     reader.onload = function (event) {
//         const json = JSON.parse(event.target.result);
//         displayContent(json);
//     };
//     reader.readAsText(file);
// }

// function displayContent(json) {
//     const contentDiv = document.getElementById('content');
//     contentDiv.innerHTML = '<pre>' + JSON.stringify(json, null, 2) + '</pre>';

    
// }

document.getElementById('fileInput').addEventListener('change', loadFile);

function loadFile(event) {
    const file = event.target.files[0];
    const reader = new FileReader();
    reader.onload = function(event) {
        const json = JSON.parse(event.target.result);
        displayContent(json);
    };
    reader.readAsText(file);
    // console.log(reader.readAsText(file));
}

function displayContent(json) {

    readFromText(json);
}