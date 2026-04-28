import { setBackground } from "./background.js";

window.onload = () => {
    setBackground();
};

const backButton = document.getElementById("back-button");
backButton.addEventListener("click", (e) => {
    window.location = "home.html";
})