mkdir -p site
cp ../ESP8266-BMS-Generator-Controller-Site/index.html site/index.html

sed -i '' '/<script src="index.js"><\/script>/{
    s/<script src="index.js"><\/script>//g
    a\
    <script>
    r ../ESP8266-BMS-Generator-Controller-Site/index.js
    a\
    </script>
}' site/index.html

sed -i '' '/<link rel="stylesheet" href="index.css">/{
    s/<link rel="stylesheet" href="index.css">//g
    a\
    <style>
    r ../ESP8266-BMS-Generator-Controller-Site/index.css
    a\
    </style>
}' site/index.html

npx html-minifier --collapse-whitespace --remove-comments --remove-optional-tags --remove-redundant-attributes --remove-tag-whitespace --use-short-doctype --minify-css true --minify-js true site/index.html -o site/index.min.html

xxd -i site/index.min.html > site/html.h
