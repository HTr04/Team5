# monkey_bot


ssh-keygen -t ed25519 -C "your_email@example.com"
Copy the public key:

bash
Copy code
cat ~/.ssh/id_ed25519.pub
Add it to GitHub → Settings → SSH and GPG keys → New SSH key.

Change your remote URL to SSH:

bash
Copy code
git remote set-url origin git@github.com:colin-szeto/monkey_bot.git
