"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database
"""

from .accounts import (
    get_all_email,
    create_account,
    get_all_accounts,
    get_one_account,
    get_id_from_email,
    get_hashed_password,
    update_one_account,
    delete_one_account
)

from .families import (
    main_id_to_family_id,
    create_family,
    find_family,
    get_all_families,
    get_one_family,
    update_one_family,
    delete_one_family
)

from .members import (
    create_member,
    get_all_members,
    get_one_member,
    update_one_member,
    delete_one_member
)

from .authentication import (
    create_session,
    delete_session,
    check_current_user,
    change_password,
    get_login_session,
    cleanup_login_sessions,
    record_auto_login
)

from .status import (
    create_home_status,
    get_home_status,
    get_latest_home_status,
    delete_latest_home_status,
    create_health_status,
    get_health_status,
    get_latest_health_status,
    delete_latest_health_status,
    create_active_status,
    get_active_status,
    get_latest_active_status,
    delete_latest_active_status,
    get_mental_status,
    get_latest_mental_status,
    delete_latest_mental_status,
    get_mental_reports,
    get_latest_mental_reports,
    delete_latest_mental_reports
)

from .notifications import (
    create_notification,
    get_new_notifications,
    get_all_notifications,
    get_one_notification,
    check_read_notification,
    delete_notification
)

from .messages import (
    create_message,
    get_new_received_messages,
    get_all_received_messages,
    get_all_sent_messages,
    get_one_message,
    check_read_message,
    delete_message
)

from .tools import (
    get_all_master_region,
    get_all_sub_region,
    get_news,
    create_settings,
    get_settings,
    update_settings,
    delete_settings,
    add_background,
    get_backgrounds,
    get_latest_background,
    get_one_background,
    delete_background
)
