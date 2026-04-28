import { setBackground } from "./background.js";

const ville = document.querySelector(".city")

let api_key = ""

fetch('/api/config')
    .then(response => response.json())
    .then(data => {
        api_key = data.api_key;
    })
    .catch(err => console.error("Impossible de charger la config:", err));


const submitButton = document.querySelector(".submit")

const backButton = document.querySelector(".go-back");


const sun = document.createElement("div")
sun.classList.add("sun");

for (let i = 0; i < 12; i++) {
    const ray = document.createElement("div");
    ray.classList.add("sun-ray");
    ray.style.transform = `rotate(${i * 30}deg)`;
    sun.appendChild(ray);
}

document.body.appendChild(sun);

backButton.addEventListener("click", (e) => {
    window.location = "home.html";
})

submitButton.addEventListener("click", () => {
    const cityValue = ville.value.trim();
    if (!cityValue) return;
    fetch(`https://api.openweathermap.org/data/2.5/weather?q=${cityValue}&appid=${api_key}&units=metric&lang=fr`)
        .then(response => response.json())
        .then(data => {
            console.log(data)

            if (data.cod === 200 || data.cod === "200") {
                const temperature = data.main.temp;
                const description = data.weather[0].description;

                const textArea = document.querySelector(".weather-result");
                textArea.value = `Il fait ${temperature} °C avec ${description}`;

            } else {
                const message = data.message || "Ville non trouvée";
                alert(message);
            }
        })
        .catch(error => {
            console.error("Erreur API:", error);
            alert("Une erreur est survenue lors de la récupération de la météo");
        });
})



window.onload = () => {
    setBackground();
};