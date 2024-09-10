# Git使用总结

## 远程库与本地库的交互操作

### 关联

1. **通用方法**：先建立Github远程仓库，再建立相关联的本地库

```
# 1、在github上创建项目（需要包括README.md，即不要创建空项目）

# 2、使用git clone https://github.com/xxxxxxx/xxxxx.git 克隆到本地
```

注意：当选取某仓库中的**特定分支**进行clone时，需在命令中加入分支信息如下

```
git clone -b [branch name] git@github.com:...
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

- 可查看当前代码状态，改动，所在分支，是否有代码冲突等


```
$ git status
```

- 查询当前文件修改内容

```
$ git diff
```



### 忽略某目录或文件上传

1. 先查看能被上传的文件

   ```
   git status
   ```

2. **在总目录创建一个`.gitignore`文件**

   ```
   touch .gitignore
   ```

3. 此时在目录中新建了一个`.gitignore`文件，在其中加入你要忽略的文件，忽略规则举例如下：

   ```
   target //忽略这个target目录
   angular.json //忽略这个angular.json文件
   log/* //忽略log下的所有文件
   css/*.css //忽略css目录下的所有.css文件
   ```

4. 保存退出，此时再执行`git status`，就查看不到上述文件了



### 撤销操作

#### git reset

三个常用模式：

1. **`--soft`**：保留所有更改，撤销提交，但保持工作目录和暂存区的状态。
2. **`--mixed`**（默认模式）：保留工作目录的更改，但取消暂存区的更改。
3. **`--hard`**：彻底回退到指定提交，删除工作目录和暂存区的所有更改。



#### 撤销add

```
git reset HEAD
```

用于**取消已暂存的更改**，将已经使用 `git add` 命令暂存的文件移回到未暂存状态



#### 撤销commit

```
git reset HEAD^
```

`HEAD^` 意思是重置到当前提交的上一个提交，将当前分支回退到上一个提交，这意味着当前提交会从分支历史中移除，也可以写成 `HEAD~1`
如果进行了 2 次 commit，都想撤回，可以使用 `HEAD~2`

或者也可以写成如下形式：

```
git reset --hard bbc6688
```

其中bbc6688为当前要恢复的commit id

上述commit id可从github网页端查找，也可使用log命令查找

```
git log
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
