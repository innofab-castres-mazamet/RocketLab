export class User {
    constructor(money = 200) {
        this.money = money;
        this.data = {
            money: this.money
        };
    }

    getMoney() {
        return this.data.money;
    }

    setMoney(value) {
        this.money = value;
        this.data.money = value;
    }
}