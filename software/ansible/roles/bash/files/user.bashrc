## Generated by Ansible (bash)

## ENVIRONMENT

# User settings
umask 022
ulimit -c 0

# System variables
export HOSTNAME="$(hostname -f)"
export DOMAIN=${HOSTNAME#*.}
export SHORTHOSTNAME=${HOSTNAME%%.*}
export REMOTEHOST="$(whoami | grep '^.*(\([^:)]*\).*$' | sed 's/^.*(\([^:)]*\).*$/\1/g')"


## "PROMPT" SHELL ONLY
[ -z "$PS1" ] && return

# Prompt
if [ -n "$(which git)" ]; then
  export PS1="\[\e[35m\]@ \[\e[33m\]\D{%Y-%m-%d %H:%M:%S %z}\[\e[35m\]\n\n* \[\e[01;32m\]\u@$(hostname -f)\[\e[00;35m\]:\[\e[36m\]\w\[\e[35m\]\$(git branch 2>/dev/null | sed -n 's|^* \(.*\)$| (\1)|p')\n\$\[\e[0m\] "
else
  export PS1="\[\e[35m\]@ \[\e[33m\]\D{%Y-%m-%d %H:%M:%S %z}\[\e[35m\]\n\n* \[\e[01;32m\]\u@$(hostname -f)\[\e[00;35m\]:\[\e[36m\]\w\[\e[35m\]\n\$\[\e[0m\] "
fi

# X Display
if [ ! "${DISPLAY}" ]; then
  [ ! "${REMOTEHOST}" -o "${REMOTEHOST}" == "${SHORTHOSTNAME}" -o "${REMOTEHOST}" == "${HOSTNAME}" ] &&  export DISPLAY=:0.0 || export DISPLAY=${REMOTEHOST}:0.0
fi

# Environment
export EDITOR='vi'

# Terminal size
shopt -s checkwinsize
export COLUMNS LINES

# Shell settings
shopt -s cdspell
shopt -s dirspell
shopt -s extglob

# History settings
shopt -s cmdhist
shopt -s histappend
HISTCONTROL=ignoreboth
HISTSIZE=1000

# Coloring
if [ -n "$(which dircolors)" ]; then
  eval "$(dircolors -b)"
  alias ls='ls --color=auto'
  alias grep='grep --color=auto'
  alias fgrep='fgrep --color=auto'
  alias egrep='egrep --color=auto'
fi

# Helpers
[ -n "$(which lesspipe)" ] && eval "$(SHELL=/bin/sh lesspipe)"

# Aliases
# ... cd
alias cd..='cd ..'
alias cd-='cd -'
# ... ls
alias l1='ls -1'
alias ll='ls -l'
alias lh='ls -lh'
alias la='ls -la'
# ... rm
alias rm='rm --one-file-system'
# ... ps
alias _psl='ps wwaxf -o user,tty,pid,state,start,time,%cpu,nlwp,ni,pri,%mem,vsz,rsz,command | less -S'
# ... systemctl
alias _ss='systemctl status'
alias _sa='sudo systemctl start'
alias _so='sudo systemctl stop'
alias _sr='sudo systemctl restart'
alias _sl='sudo systemctl reload'
alias _sc='systemctl cat'
alias _sj='journalctl -u'
# ... apt
alias _au='sudo apt-get update'
_as() { apt-cache search $@ | sort ; }
alias _ap='apt-cache policy'
alias _aw='apt-cache show'
_av() { [ $# -le 0 ] && dpkg -l | sort || dpkg -l | fgrep $1 | sort ; }
alias _ai='sudo apt-get install'
alias _aii='sudo apt-get install --reinstall'
alias _ar='sudo apt-get remove --purge'
alias _aar='sudo apt-get autoremove --purge'
alias _adu='sudo apt-get dist-upgrade'
alias _aduh='sudo apt-get dist-upgrade --ignore-hold'
alias _ac='sudo apt-get clean'
# ... dpkg
alias _dl='dpkg -L'
alias _dp='sudo dpkg -P'
# ... misc
alias _dm='sudo dmesg --time-format=iso | tail -n 25'
alias _sy='(logread 2>/dev/null || sudo cat /var/log/syslog) | tail -n 25'

# Completions
[ -r /etc/bash_completion ] && . /etc/bash_completion
# ... systemctl
function _mycomp_systemctl_units {
  local cur=`_get_cword`
  COMPREPLY=( $( systemctl list-units --no-legend --no-pager --all --full --type=target,service,timer,mount,path "${cur}*" | awk '{print $1}' ) )
}
complete -F _mycomp_systemctl_units _ss
complete -F _mycomp_systemctl_units _sa
complete -F _mycomp_systemctl_units _so
complete -F _mycomp_systemctl_units _sr
complete -F _mycomp_systemctl_units _sl
complete -F _mycomp_systemctl_units _sj
# ... apt/dpkg
function _mycomp_apt_available_packages {
 local cur=`_get_cword`
 COMPREPLY=( $( apt-cache pkgnames "$cur" 2>/dev/null ) )
}
complete -F _mycomp_apt_available_packages _aw
complete -F _mycomp_apt_available_packages _ap
complete -F _mycomp_apt_available_packages _ai
function _mycomp_dpkg_installed_packages {
 local cur=`_get_cword`
 COMPREPLY=( $( dpkg --get-selections | awk '{print $1}' | grep "^$cur" ) )
}
complete -F _mycomp_dpkg_installed_packages _ar
complete -F _mycomp_dpkg_installed_packages _aar
complete -F _mycomp_dpkg_installed_packages _aii
complete -F _mycomp_dpkg_installed_packages _dl
complete -F _mycomp_dpkg_installed_packages _dp

# Additional settings
[ -d ~/.bashrc.d ] && for file in ~/.bashrc.d/*; do [ -r "${file}" ] && source ${file}; done

# Cleanup
unset file

# Welcome message
echo ""
echo "MACHINE: ${HOSTNAME} (${USER})"
echo "SYSTEM:  $(uname) $(uname -r)"
echo "DISPLAY: ${DISPLAY}"
echo "TMPDIR:  ${TMPDIR}"
echo ""
cd
