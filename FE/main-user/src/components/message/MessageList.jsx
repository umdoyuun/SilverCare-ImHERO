import React, { useState, useEffect } from 'react';
import { useMessageStore } from '../../store/messageStore';
import './Message.css';

export default function MessageList({ onSelectUser }) {
    const { receivableUsers, fetchReceivableUsers, selectUser, selectedUser } = useMessageStore();
    const [isChatOpen, setIsChatOpen] = useState(false);
    const [clickedUser, setClickedUser] = useState(null);

    // 명단 불러오기
    useEffect(() => {
        fetchReceivableUsers();
    }, []);

    const handleUserSelect = (user) => {
        selectUser(user);
        setIsChatOpen(true);
    };

    return (
        <div className="user-buttons">
            {receivableUsers.map((user) => (
                <button 
                    key={user?.user_id || Math.random()}
                    className="user-button" 
                    onClick={() => {
                        if (!user?.user_id) {
                            console.error("Error: user_id is undefined", user);
                            return;
                        }
                        setClickedUser(user.user_id);
                        setTimeout(() => setClickedUser(null), 500);
                        selectUser(user);
                        onSelectUser();
                        handleUserSelect(user);
                }}>
                    {user?.name || "이름 없음"}
                </button>
            ))}
        </div>
    );
};