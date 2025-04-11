let environments = {};

// Environment 값 불러오기
export const loadEnvironments = async () => {
    try {
        const response = await fetch('/env');
        environments = await response.json();
    }
    catch (error) {
        console.log("환경 변수 로드 실패:", error);
    }
};

// Environment 값 반환하기
export const getEnvironments = (key) => environments[key];