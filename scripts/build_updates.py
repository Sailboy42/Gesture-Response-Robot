#!/usr/bin/env python3
"""
Build script: read markdown files from updates/, convert to HTML, and inject into a built site/ folder.

Behavior:
- Reads updates/*.md (expects filenames starting with YYYY-MM-DD or containing a date)
- Converts them to HTML (python-markdown)
- Copies site assets (index.html, css/, js/, assets/) into site/
- Replaces the text between <!-- UPDATES-START --> and <!-- UPDATES-END --> in site/index.html with generated articles
"""
import glob
import os
import shutil
import sys
from datetime import datetime

try:
    import frontmatter
except Exception:
    frontmatter = None
try:
    import markdown
except Exception:
    markdown = None

ROOT = os.path.dirname(os.path.dirname(__file__)) if os.path.basename(__file__)=="build_updates.py" else os.getcwd()
UPDATES_DIR = os.path.join(ROOT, 'updates')
SITE_DIR = os.path.join(ROOT, 'site')
SRC_INDEX = os.path.join(ROOT, 'index.html')
SITE_INDEX = os.path.join(SITE_DIR, 'index.html')

if not os.path.exists(UPDATES_DIR):
    print('No updates directory found. Create updates/ and add markdown files named like 2025-11-17-my-update.md')
    # still create a site copy

# collect markdown files
md_files = sorted(glob.glob(os.path.join(UPDATES_DIR, '*.md')), reverse=True)
entries = []
for path in md_files:
    name = os.path.basename(path)
    # default date from filename (YYYY-MM-DD prefix)
    date_str = None
    if len(name) >= 10 and name[4] == '-' and name[7] == '-':
        date_str = name[:10]
    else:
        # fallback: use file mtime
        mtime = os.path.getmtime(path)
        date_str = datetime.utcfromtimestamp(mtime).strftime('%Y-%m-%d')

    with open(path, 'r', encoding='utf8') as fh:
        raw = fh.read()

    # parse front-matter if available
    meta = {}
    content = raw
    if frontmatter:
        try:
            post = frontmatter.loads(raw)
            meta = post.metadata or {}
            content = post.content
            # if front-matter has date, prefer it
            if 'date' in meta:
                try:
                    # accept date as string
                    date_str = str(meta['date'])
                except Exception:
                    pass
        except Exception:
            meta = {}
            content = raw

    # render markdown to HTML (ensure markdown package is present)
    if markdown:
        html_body = markdown.markdown(content, extensions=['fenced_code', 'codehilite'])
    else:
        # fallback: escape minimal markup
        html_body = '<pre>' + content.replace('<','&lt;').replace('>','&gt;') + '</pre>'

    # build entry metadata
    title = meta.get('title')
    author = meta.get('author')
    summary = meta.get('summary')
    image = meta.get('image')
    entries.append({'date': date_str, 'html': html_body, 'filename': name, 'title': title, 'author': author, 'summary': summary, 'image': image})

# build snippet
if entries:
    parts = []
    for e in entries:
        title_html = f"<h4>{e['title']}</h4>" if e.get('title') else ''
        author_html = f"<p class=\"meta\">By {e['author']}</p>" if e.get('author') else ''
        summary_html = f"<p class=\"summary\">{e['summary']}</p>" if e.get('summary') else ''
        image_html = f"<div class=\"update-image-wrap\"><img class=\"update-image\" src=\"{e['image']}\" alt=\"update image\"></div>" if e.get('image') else ''
        parts.append(f"<article class=\"update\"><time datetime=\"{e['date']}\">{e['date']}</time>\n{title_html}{author_html}{summary_html}{image_html}{e['html']}</article>")
    snippet = '\n'.join(parts)
else:
    snippet = '<p>No updates yet. Add markdown files to the updates/ directory.</p>'

# prepare site directory: copy files (index.html, css, js, assets)
if os.path.exists(SITE_DIR):
    shutil.rmtree(SITE_DIR)
os.makedirs(SITE_DIR, exist_ok=True)

# copy index.html
if not os.path.exists(SRC_INDEX):
    print('Source index.html not found at', SRC_INDEX)
    sys.exit(1)
shutil.copy2(SRC_INDEX, SITE_INDEX)

# copy folders if they exist
for d in ('css', 'js', 'assets'):
    src = os.path.join(ROOT, d)
    dst = os.path.join(SITE_DIR, d)
    if os.path.exists(src):
        shutil.copytree(src, dst)

# inject snippet into site/index.html between markers
with open(SITE_INDEX, 'r', encoding='utf8') as fh:
    page = fh.read()
start_marker = '<!-- UPDATES-START -->'
end_marker = '<!-- UPDATES-END -->'
if start_marker in page and end_marker in page:
    before, rest = page.split(start_marker, 1)
    inside, after = rest.split(end_marker, 1)
    new_page = before + start_marker + '\n' + snippet + '\n' + end_marker + after
    with open(SITE_INDEX, 'w', encoding='utf8') as fh:
        fh.write(new_page)
    print('site built at', SITE_DIR)
else:
    print('Markers not found in', SITE_INDEX)
    sys.exit(2)

print('Done.')
