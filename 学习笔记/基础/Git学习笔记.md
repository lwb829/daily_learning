# Git使用总结

## 远程库与本地库的交互操作

### 关联

1. **通用方法**：先建立Github远程仓库，再建立相关联的本地库

```
# 1、在github上创建项目（需要包括README.md，即不要创建空项目）

# 2、使用git clone https://github.com/xxxxxxx/xxxxx.git 克隆到本地
```

2. 一般方法：先建立本地库，再关联远程库

```
# 1. 先本地新建仓库
git init
# 2. 在github上创建空项目（不要添加任何东西，包括README.md）
# 3. 根据github提示关联本地库
# 待下次新建仓库时更新
```

### 同步

1. 本地库更新到远程库

```
git add -A  # -A表示将本地的所有更改都添加到暂存区，也可以指定某个文件
git add ××  # ××表示文件名
git commit -m "xxx"  # ×××为commit message，该步骤不可缺少   
git push origin main
# 首次push需要加'-u'
git push -u origin main
```

**注：此处的origin为远程库的main分支，main就是本地的main分支**

2. 远程库更新到本地库

```
git pull origin main
```



### 文件（夹）删除

Github仓库中的**某一分支可以直接删除**，但是某一分支中的**某一文件无法直接删除**，必须经过clone到本地库中进行操作

1. 查看远程仓库所有文件夹

```
dir
```

2. 删除远程仓库某文件夹

```
git rm -r --cached ××
```

3. 本地删除操作：

```
$ git rm ××  # ××为当前分支中要删除的文件名
```



### 查看文件修改状态

可查看当前代码状态，改动，所在分支，是否有代码冲突等

```
$ git status
```



### 撤销操作

#### 撤销commit、不撤销git add .

```
git reset --soft HEAD^
```

`HEAD^` 意思是上一个版本，也可以写成 `HEAD~1`
如果进行了 2 次 commit，都想撤回，可以使用 `HEAD~2`



#### 撤销 commit、撤销 git add .

```
 git reset --hard HEAD^
```



## 分支相关操作

### 创建本地分支

```
git branch [branchname]
```



### 查看分支

```
# 查看当前所在分支
git branch
# 查看所有分支
git branch -a
```



### 切换本地分支

```
git checkout [branchname]
```

- 可以创建分支与切换分支同步操作

  ```
  git checkout -b [branchname]
  ```



### 删除本地分支

```
git branch -d [branchname]
```

如果分支尚未合并，您可能需要使用 -D 标志来强制删除本地分支

```
git branch -D [branchname]
```



### 删除远程分支

**确保本地分支已经被删除，若先删除远程分支，可能会导致本地源文件被删除**

```
git push origin --delete [branchname]
```

其中，origin 是远程仓库的名称，branch_name 是要删除的分支的名称
