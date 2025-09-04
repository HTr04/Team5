# monkey_bot

1. Generate SSH key on the Pi:

```ssh-keygen -t ed25519 -C "your_email@example.com"```


2. Copy the public key:

```cat ~/.ssh/id_ed25519.pub```

3. Add it to GitHub → Settings → SSH and GPG keys → New SSH key.

4. Change your remote URL to SSH:

```git remote set-url origin git@github.com:colin-szeto/monkey_bot.git```
