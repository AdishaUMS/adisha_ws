const express   = require('express');
const path      = require('path');
const app       = express();
const port      = process.env.PORT || 3000;
const html_path = path.join(__dirname, '..', 'web', 'index.html');

/* Response on a request */
app.get('/', (req, res) => {
  res.sendFile(html_path);
});

/* Listen to port */
app.listen(port, () => {
  console.log(`Server is running on port ${port}`);
});