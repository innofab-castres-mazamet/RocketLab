import express from 'express';
import path from 'path';
import { fileURLToPath } from 'url';
import dotenv from 'dotenv';
import mysql from 'mysql2/promise';

dotenv.config();

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const app = express();
const PORT = process.env.PORT;

app.use(express.json());

const db = mysql.createPool({
    host: process.env.DB_HOST,
    port: process.env.DB_PORT,
    user: process.env.DB_USER,
    password: process.env.DB_PASSWORD,
    database: process.env.DB_NAME,
});

app.use('/style', express.static(path.join(__dirname, 'style')));
app.use('/script', express.static(path.join(__dirname, 'script')));
app.use('/lib', express.static(path.join(__dirname, 'lib')));
app.use('/page', express.static(path.join(__dirname, 'page')));
app.use(express.static(path.join(__dirname)));


app.get('/api/config', (req, res) => {
    res.json({
        api_key: process.env.API_KEY
    });
});


app.get('/api/rockets', async(req, res) => {
    try{
        const [rows] = await db.query('SELECT * FROM Rocket');
        res.json(rows);
    }
    catch(error){
        console.error(error);

        res.status(500).json({
            error: 'Erreur serveur'
        });
    }
});


app.get('/api/rockets/:id', async(req, res) => {
    try{
        const [rows] = await db.query('SELECT * FROM Rocket WHERE idRocket = ?', [req.params.id]);

        if (rows.length === 0) {
            return res.status(404).json({
                errorr: 'Rocket introuvable'
            });
        }
        res.json(rows[0])
    }
    catch(error){
        console.error(error);

        res.status(500).json({
            error: 'Erreur serveur'
        });
    }
});



app.get('/', (req, res) => {
    res.redirect('/home.html');
});

app.get('/:pageName', (req, res) => {
    const pageName = req.params.pageName;
    const filePath = path.join(__dirname, 'page', `${pageName}`);
    
    res.sendFile(filePath, (err) => {
        if (err) {
            res.status(404).send('Page non trouvée');
        }
    });
});

app.listen(PORT, () => {
    console.log(`serveur lancé sur http://localhost:${PORT}/`);
});
