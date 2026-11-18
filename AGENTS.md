# Правила проекта CG_Scale

## Синхронизация с GitHub

Репозиторий: https://github.com/permyakov344-png/CG_Scale (private), ветка `main`.

- Быстро: двойной клик `tools\sync-now.cmd`.
- Или скажи агенту: «синхронизируй с GitHub».
- Вручную:

  ```powershell
  git add -A
  git commit -m "краткое описание"
  git pull --rebase origin main
  git push origin main
  ```

- Не делать `git push --force` и `git reset --hard`.
- Еженедельная задача Windows: `CG_Scale weekly backup`, понедельник 12:00.
- Лог синхронизации: `..\sketch_CG_backups\weekly-backup.log`.

## Сборка и прошивка

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 "C:\Users\user\Documents\Arduino\sketch_CG"
arduino-cli upload  --fqbn esp32:esp32:esp32 -p COM21 "C:\Users\user\Documents\Arduino\sketch_CG"
```

Перед прошивкой делать бэкап `.ino` в `..\sketch_CG_backups\`.

## Документация

- `README.md` — сборка, управление, экраны, история.
- `HISTORY.md` — короткая история по версиям.
- `ПРОЕКТ_СТАТУС.md` — подробная история и чек-листы.
