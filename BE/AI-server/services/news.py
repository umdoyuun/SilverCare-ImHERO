# services/news.py
import aiohttp
import asyncio
from datetime import datetime
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from models import News
import logging
from typing import Dict, List, Optional
from pydantic import BaseModel
from datetime import date

logger = logging.getLogger(__name__)

class NewsBase(BaseModel):
    title: str
    link: str
    pub_date: datetime
    image_url: Optional[str] = None
    category: str

    class Config:
        from_attributes = True

class NewsService:
    def __init__(self, api_key: str):
        self.api_key = api_key
        self.categories = [
            "business", "entertainment", "environment", "health",
            "politics", "science", "sports", "technology"
        ]
        self.is_updating = False
        self.last_update = None

    async def update_news(self):
        if self.is_updating:
            return
        
        try:
            self.is_updating = True
            current_time = datetime.now()
            if self.last_update and (current_time - self.last_update).total_seconds() < 24 * 60 * 60:
                return

            news_data = await self.fetch_all_news()
            if not news_data:
                return

            from main import db_manager
            db = db_manager.db
            
            try:
                for category, news_list in news_data.items():
                    if not news_list:
                        continue
                    
                    for news in news_list:
                        try:
                            date_str = news['pubDate'].split(' ')[0]
                            pub_date = datetime.strptime(date_str, '%Y-%m-%d').date()
                            
                            existing_news = db.query(News).filter(News.link == news['link']).first()
                            if existing_news:
                                continue

                            news_item = News(
                                title=news['title'],
                                link=news['link'],
                                pub_date=pub_date,
                                image_url=news.get('image_url'),
                                category=category,
                                created_at=current_time  #
                            )
                            db.add(news_item)
                            
                        except Exception as e:
                            logger.error(f"Error processing news item: {str(e)}")
                            continue
                
                db.commit()
                self.last_update = current_time
                logger.info(f"News data successfully updated at {current_time}")
                
            except Exception as e:
                db.rollback()
                raise
            
        except Exception as e:
            logger.error(f"Error updating news: {str(e)}")
        finally:
            self.is_updating = False

    async def fetch_all_news(self) -> Dict[str, List[Dict]]:
        async with aiohttp.ClientSession() as session:
            news_dict = {}
            for category in self.categories:
                result = await self.fetch_category_news(session, category)
                if result:
                    news_dict[category] = result
                await asyncio.sleep(1.1)
            return news_dict

    async def fetch_category_news(self, session: aiohttp.ClientSession, category: str) -> Optional[List[Dict]]:
        url = f"https://newsdata.io/api/1/latest?apikey={self.api_key}&country=kr&language=ko&category={category}"
        try:
            async with session.get(url) as response:
                if response.status == 200:
                    data = await response.json()
                    return data.get('results', [])
                else:
                    logger.error(f"News API HTTP error for {category}: {response.status}")
                    return None
        except Exception as e:
            logger.error(f"Error fetching news for {category}: {str(e)}")
            return None