#! /bin/sh

wget -O data/dygraph.min.js https://dygraphs.com/2.2.1/dist/dygraph.min.js
wget -O data/dygraph.css https://dygraphs.com/2.2.1/dist/dygraph.css

cp data/dygraph.min.js data/dygraph.min.js.t
gzip data/dygraph.min.js.t
mv data/dygraph.min.js.t.gz data/dygraph.min.js.gz

cp data/dygraph.css data/dygraph.css.t
gzip data/dygraph.css.t
mv data/dygraph.css.t.gz data/dygraph.css.gz
