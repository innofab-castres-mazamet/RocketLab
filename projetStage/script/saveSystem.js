import {User} from "../Classes/User.js";

export function saveUser(user) {
    localStorage.setItem("userData", JSON.stringify(user.data));
}

export function loadUser() {
    const savedData = localStorage.getItem("userData");
    if (savedData) {
        const data = JSON.parse(savedData);
        return new User(data.money);
    }
    return new User();
}
