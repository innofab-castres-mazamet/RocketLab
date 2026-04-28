import { setBackground } from "./background.js";

const backButton = document.querySelector("#back-button");
backButton.addEventListener("click", () => {
    window.location.href = "home.html";
})



function time() {
    const now = new Date();

    const optionsDate = {
        weekday: 'long',
        year: 'numeric',
        month: 'long',
        day: 'numeric'
    };

    const date = now.toLocaleDateString('fr-FR', optionsDate);
    const heure = now.toLocaleTimeString('fr-FR');

    document.getElementById('time').textContent = `${date} - ${heure}`;
}

time();

setInterval(time, 1000);


const startTimeButton = document.querySelector("#time-start");
const display = document.querySelector("#display");

let intervalId = null
let seconds = 0;
let minutes = 0;
let heures = 0;
startTimeButton.addEventListener("click", () => {
    if(intervalId === null) {
        intervalId = setInterval(() => {
            seconds ++;
            display.textContent = `${seconds} s`;
            if (seconds === 60) {
                minutes ++;
                seconds = 0
            }
            if (minutes === 60) {
                heures ++;
                minutes = 0;
            }

            if (heures >= 1) {
                display.textContent = `${heures} h ${minutes} min ${seconds} s`;
            } else if (minutes >= 1) {
                display.textContent = `${minutes} min ${seconds} s`;
            }

        }, 1000);
        startTimeButton.textContent = "Arrêter";
    } else{
        clearInterval(intervalId);
        intervalId = null
        startTimeButton.textContent = "Démarrer"
    }
});

const resetTimeButton = document.getElementById('time-reset');

resetTimeButton.addEventListener('click', () => {
    clearInterval(intervalId);
    intervalId = null;
    seconds = 0;
    display.textContent = `${seconds} s`;
    startTimeButton.textContent = 'Démarrer';
});


const startTimerButton = document.getElementById('timer-start');
const resetTimerButton = document.getElementById('timer-reset');
const timerResult = document.getElementById('timer-result');
const timerInput = document.getElementById('timer-time');
let timerInterval = null;
let timerSecondsRemaining = 0;

function updateTimerDisplay() {
    const mins = Math.floor(timerSecondsRemaining / 60);
    const secs = timerSecondsRemaining % 60;
    timerResult.textContent = `${mins} min ${secs < 10 ? '0' : ''}${secs} s`;
}

startTimerButton.addEventListener("click", () => {
    if (timerInterval === null) {
        if (timerSecondsRemaining <= 0) {
            const minutesInput = parseFloat(timerInput.value);
            if (isNaN(minutesInput) || minutesInput <= 0) {
                alert("Veuillez entrer une durée valide en minutes.");
                return;
            }
            timerSecondsRemaining = Math.round(minutesInput * 60);
        }

        updateTimerDisplay();
        timerInterval = setInterval(() => {
            timerSecondsRemaining--;
            updateTimerDisplay();

            if (timerSecondsRemaining <= 0) {
                clearInterval(timerInterval);
                timerInterval = null;
                startTimerButton.textContent = "Démarrer";
                alert("Temps écoulé !");
            }
        }, 1000);
        startTimerButton.textContent = "Arrêter";
    } else {
        clearInterval(timerInterval);
        timerInterval = null;
        startTimerButton.textContent = "Démarrer";
    }
});

resetTimerButton.addEventListener("click", () => {
    clearInterval(timerInterval);
    timerInterval = null;
    timerSecondsRemaining = 0;
    timerResult.textContent = "";
    startTimerButton.textContent = "Démarrer";
    timerInput.value = "";
});

window.onload = () => {
    setBackground();
};