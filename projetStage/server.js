import express from 'express';
import path from 'path';
import { fileURLToPath } from 'url';
import dotenv from 'dotenv';

dotenv.config();

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const app = express();
const PORT = 3000;

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
