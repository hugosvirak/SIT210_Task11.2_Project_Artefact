const http = require('http')
const express = require('express')
const WebSocket = require('ws')
const app = express()
const port = 80

const bodyParser = require('body-parser');
app.use(bodyParser.urlencoded({ extended: false }));

app.use('/', express.static('public'));

const server = http.createServer(app);
const wss = new WebSocket.Server({ server })

wss.on('connection', ws => {
  ws.send("Connected. Listening to server updates.");
  ws.on('close', function close() {

  });
})

function sendToAllWebClients(data) {
  wss.clients.forEach(function each(client) {
	  if (client.readyState === WebSocket.OPEN) {
		client.send(data, { binary: 0 });
	  }
	});
}

app.get('/new_sample', (req, res) => {
	if (req.query.value) {
		sendToAllWebClients("new_sample: Reading: " + req.query.value);
		res.send();
	}
	
	res.status(400).send();
})

server.listen(port, () => {
  console.log(`Listening at http://localhost:${port}`)
})