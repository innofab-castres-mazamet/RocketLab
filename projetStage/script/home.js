import { setBackground } from "./background.js";

const weatherButton = document.querySelector("#weather-button");
weatherButton.addEventListener("click", (e) => {
    window.location = ("weather.html");
})

const recordsButton = document.querySelector("#records-button");
recordsButton.addEventListener("click", (e) => {
    window.location = ("records.html");
})

const timeButton = document.querySelector("#time-button");
timeButton.addEventListener("click", (e) => {
    window.location = ("time.html");
})

window.onload = () => {
    setBackground();
};