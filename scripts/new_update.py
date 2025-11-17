#!/usr/bin/env python3
"""
scaffold a new update markdown file in updates/ with today's date and a simple front-matter template
Usage: python scripts/new_update.py "Optional Title Here"
"""
import os
import sys
from datetime import datetime

def slugify(s):
    return ''.join(c if c.isalnum() else '-' for c in s).strip('-').lower()

if __name__ == '__main__':
    title = 'New update'
    if len(sys.argv) > 1:
        title = sys.argv[1]
    today = datetime.now().strftime('%Y-%m-%d')
    filename = f"{today}-{slugify(title)}.md"
    updates_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'updates')
    os.makedirs(updates_dir, exist_ok=True)
    path = os.path.join(updates_dir, filename)
    if os.path.exists(path):
        print('File already exists:', path)
        sys.exit(1)
        content = f"""---
    # optional front-matter fields: title, author, date, summary, image
    title: {title}
    author: 
    date: {today}
    summary: A short summary of this update.
    image: 
    ---

    # {title}

    Describe the update here.
    """
    with open(path, 'w', encoding='utf8') as fh:
        fh.write(content)
    print('Created', path)
