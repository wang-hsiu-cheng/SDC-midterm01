"""把 deck_template.html 建成可直接寄送的單一 HTML（圖片以 data URI 嵌入）。

用法：python3 build_deck.py <template> <figs_dir> <out_dir>
輸出：presentation.html（主頁 9 張，最後一張是深色結尾）、presentation_with_backup.html（主頁＋備用頁 3 張）
"""
import base64
import re
import sys

tpl_path, figs_dir, out_dir = sys.argv[1:4]
tpl = open(tpl_path, encoding='utf-8').read()


def embed(html):
    def sub(m):
        data = open('%s/%s' % (figs_dir, m.group(1)), 'rb').read()
        return 'data:image/png;base64,' + base64.b64encode(data).decode('ascii')
    return re.sub(r'@@([A-Za-z0-9_]+\.png)', sub, html)


def drop_backup(html):
    return re.sub(r'<!-- backup [A-Z]:.*?</section>\s*', '', html, flags=re.S)


full = embed(tpl)
main = embed(drop_backup(tpl))
open(out_dir + '/presentation_with_backup.html', 'w', encoding='utf-8').write(full)
open(out_dir + '/presentation.html', 'w', encoding='utf-8').write(main)
print('main slides:', main.count('<section class="slide'), ' with backup:', full.count('<section class="slide'))
