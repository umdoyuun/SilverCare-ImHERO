import { useState, useCallback } from "react";

export function useMainHttp() {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const request = useCallback(
    async (url, method = "GET", body = null, headers = {}) => {
      setLoading(true);
      setError(null);

      try {
        console.log(`📡 요청 시작: ${method} ${url}`, body)
        
        const response = await fetch(url, {
          method,
          body: body ? JSON.stringify(body) : null,
          headers: {
            "Content-Type": "application/json",
            ...headers,
          },
          credentials: "include", // 필요한 경우 포함
        });

        console.log("📡 HTTP 응답 상태 코드:", response.status);

        const resData = await response.json().catch(() => null); // JSON 변환 실패 방지

        console.log("📡 응답 JSON 데이터:", resData);
        
        if (!response.ok) {
          if (response.status === 403) {
            console.error("세션 만료", response.error);
            sessionStorage.removeItem("loginUserInfo");
            window.location.href = "/";
            return {
              success: false,
              error: {
                type: response.status,
                message: "세션이 만료되었습니다. 다시 로그인해주세요.",
              },
              data: null,
            };
          }
          return {
            success: false,
            error: resData?.detail || "요청 실패",
            data: null,
          };
        }

        return { success: true, data: resData };
      } catch (err) {
        setError(err.message);
        return { success: false, error: err.message, data: null };
      } finally {
        setLoading(false);
      }
    },
    []
  );

  return { request, loading, error };
}