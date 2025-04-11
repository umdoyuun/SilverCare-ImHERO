// 필요한 라이브러리 불러오는 부분
var express = require('express');
var dotenv = require('dotenv');
var path = require('path');
const history = require('connect-history-api-fallback');
var cookieParser = require('cookie-parser');
var logger = require('morgan');

// ENV 불러오기
dotenv.config();

// Express 설정
var app = express();

// EJS View 설정
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'ejs');

// Express 확장 설정
app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());

// React를 위한 환경 변수
app.get('/env', (req, res) => {
  res.json({
    DEV_API_URL: process.env.REACT_APP_DEV_API,
    MAIN_API_URL: process.env.REACT_APP_MAIN_API,
    IMAGE_API_URL: process.env.REACT_APP_IMAGE_API,
    DEV_KEY: process.env.REACT_APP_DEV_KEY,
    MAIN_KEY: process.env.REACT_APP_MAIN_KEY,
    MODE: process.env.NODE_ENV,
  });
});

// React 파일 연결하기
app.use(history());
app.use('/', express.static(path.join(__dirname, 'public')));

// React 연결하기
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, 'public/index.html'));
});

module.exports = app;
