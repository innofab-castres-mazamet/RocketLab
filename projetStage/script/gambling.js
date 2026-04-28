
import {User} from "../Classes/User.js";
import { setBackground } from "./background.js";
import {saveUser, loadUser} from "./saveSystem.js";
let user = loadUser();

const backButton = document.querySelector(".go-back");
backButton.addEventListener("click", () => {
    window.location = ("home.html");
})

export function getsBiggerWhenAimed(){
    const spinButton = document.getElementById("spin-button");
    spinButton.style.transform = "scale(1.1)";
}

export function getsSmallerWhenAimed(){
    const spinButton = document.getElementById("spin-button");
    spinButton.style.transform = "scale(1)";
}


const images = [
    "🍒",
    "🍋",
    "🍊",
    "⭐",
    "💎"
];



let money = user.getMoney();
const placeForMoney = document.getElementById('value-money');
placeForMoney.textContent = money + " €";

let timeLeft = 600;
const placeForTime = document.getElementById('value-time');

function updateTimerDisplay() {
    const minutes = Math.floor(timeLeft / 60);
    const seconds = timeLeft % 60;
    placeForTime.textContent = `${minutes}:${seconds < 10 ? '0' : ''}${seconds}`;
}


function getSymbol(index, firstResult) {
    if (index === 1 && firstResult) {
        return Math.random() < 0.6 ? firstResult : images[Math.floor(Math.random() * images.length)];
    }
    return images[Math.floor(Math.random() * images.length)];
}

function updateReelContent(reel, content) {
    reel.innerHTML = `<div class="reel-content"><div class="symbol">${content}</div></div>`;
}

export function spin() {
    if (money >= 50) {
        const reels = [
            document.getElementById("reel1"),
            document.getElementById("reel2"),
            document.getElementById("reel3")
        ];

        const results = [];
        let completedReels = 0;
        const messageElement = document.getElementById("message");
        messageElement.textContent = "";
        messageElement.className = "";

        const numExtraSymbols = 15;

        reels.forEach((reel, index) => {
            const reelContent = document.createElement('div');
            reelContent.className = 'reel-content';

            const currentSymbolElement = reel.querySelector('.symbol:last-child');
            const currentSymbol = currentSymbolElement ? currentSymbolElement.textContent : reel.textContent.trim();
            reelContent.innerHTML = `<div class="symbol">${currentSymbol}</div>`;

            for (let i = 0; i < numExtraSymbols; i++) {
                reelContent.innerHTML += `<div class="symbol">${getSymbol()}</div>`;
            }

            const result = getSymbol(index, results[0]);
            results[index] = result;
            reelContent.innerHTML += `<div class="symbol">${result}</div>`;

            reel.innerHTML = '';
            reel.appendChild(reelContent);

            setTimeout(() => {
                reelContent.classList.add("spin-animation", "blur");
                const symbolHeight = 120;
                const totalMove = (numExtraSymbols + 1) * symbolHeight;
                reelContent.style.transform = `translateY(-${totalMove}px)`;
            }, index * 200);

            setTimeout(() => {
                reelContent.classList.remove("blur", "spin-animation");
                reelContent.style.transform = "none";
                updateReelContent(reel, result);

                completedReels++;
                if (completedReels === reels.length) {
                    checkWin(results);
                }
            }, 2000 + index * 200);
        });
    }
    else{
        const message = document.getElementById("message");
        message.textContent = "Pas assez d'argent !"
        message.className = "not-enough-money";
    }

}

function checkWin(results) {
    money -= 50;
    const messageElement = document.getElementById("message");
    if (results[0] === results[1] && results[1] === results[2]) {
        money += 500;
        messageElement.textContent = "Jackpot !!!";
        messageElement.className = "jackpot";
    }
    else if (results[0] === results[1] || results[1] === results[2] || results[0] === results[2]) {
        money += 100;
        messageElement.textContent = "Bien !";
        messageElement.className = "ok";
    }
    else {
        messageElement.textContent = "Dommage. Essaie encore !";
        messageElement.className = "dommage";
    }
    user.setMoney(money);
    saveUser(user);
    placeForMoney.textContent = money + " €";
}

window.onload = () => {

    saveUser(user)

    setBackground();

    updateTimerDisplay();

    setInterval(() => {
        if (timeLeft > 0) {
            timeLeft--;
        } else {
            money += 100;
            user.setMoney(money);
            saveUser(user);
            placeForMoney.textContent = money + " €";
            timeLeft = 600; // Réinitialise à 10 minutes
        }
        updateTimerDisplay();
    }, 1000);

    const reels = [
        document.getElementById("reel1"),
        document.getElementById("reel2"),
        document.getElementById("reel3")
    ];

    const initialResults = [];
    reels.forEach((reel, index) => {
        const result = getSymbol(index, initialResults[0]);
        initialResults[index] = result;
        updateReelContent(reel, result);
    });

};

