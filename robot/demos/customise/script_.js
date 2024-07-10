import { readFromText } from '../../main.js';
import { readFromJSONandOBJ } from '../../main.js';


document.getElementById('fileInput').addEventListener('change', handleFiles);


function handleFiles() {
	const fileInput = document.getElementById('fileInput');
	const files = fileInput.files;
	let jsonFile;
	const objFilesContent = {};

	const promises = [];

	for (let i = 0; i < files.length; i++) {
		const file = files[i];
		const fileName = file.name;
		const fileExtension = fileName.split('.').pop().toLowerCase();

		if (fileExtension === 'json') {
			const reader = new FileReader();
			promises.push(new Promise((resolve, reject) => {
				reader.onload = function(event) {
					try {
						jsonFile = JSON.parse(event.target.result);
						resolve();
					} catch (e) {
						reject(e);
					}
				};
				reader.readAsText(file);
			}));
		} else if (fileExtension === 'obj') {
			const reader = new FileReader();
			promises.push(new Promise((resolve) => {
				reader.onload = function(event) {
					objFilesContent[fileName] = event.target.result;
					resolve();
				};
				reader.readAsText(file);
			}));
		}
	}

	Promise.all(promises).then(() => {
		

		// console.log('Parsed JSON Files:', jsonFile);
		// console.log('OBJ Files Content:', objFilesContent);

		readFromJSONandOBJ(jsonFile, objFilesContent);


	}).catch(error => {
		console.error('Error parsing files:', error);
	});
}