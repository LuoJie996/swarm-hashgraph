#!/usr/bin/python
# coding=utf-8
import fileinput
from scipy import stats
import numpy as np
#from pyemd import emd_samples
from sklearn.cluster import KMeans
import math
#-----------------------------------------------------------------------------#
#                                ByzantineFilter                              #
#-----------------------------------------------------------------------------#
def byzantineFilter(votesdict, filterFlag):
    byzantinerobot_id=[]
    if filterFlag:
        allvoteslist=[]
        for sender in votesdict.keys():
            allvoteslist.append(votesdict[sender])
        allvoteslistcopy = allvoteslist[:]
        allRobotIds = [i for i in range(len(allvoteslist))]
        allRobotIdscopy = allRobotIds[:]
        count = 0
        
        A_id =[]
        B_id =[]
        C_id =[]
        # if len(allvoteslist)<2:
        #     print("fail")
        #     return byzantinerobot_id

        mean_std = []
        for votes in allvoteslist:
            mean=np.mean(votes)
            std=np.std(votes,ddof=1)
            mean_std.append([mean,std])


        total_mean = 0 
        total_se = 1
        while True:
            count=count+1
            allrobotsIds_votes= dict(zip(allRobotIdscopy,allvoteslistcopy))
            for k in A_id:
                del allrobotsIds_votes[k]
            allvotes=[]

            #print("count: "+str(count))

            for votes in allvoteslist:
                allvotes.extend(votes)
            total_mean=np.mean(allvotes)
            total_std=np.std(allvotes,ddof=1)
            total_se=total_std/math.sqrt(len(allvotes))

            different  = False
            dp=[]
            A=False
            B=False
            C=False

            #检验所有样本是否来自同一分布
            #方案A: 用kstest检验循环两个样本是否来自同一分布
            for k in allrobotsIds_votes.keys():
                d,p=stats.kstest(allrobotsIds_votes[k],allvotes)
                dp.append([d,p])
                if p < 0.05:
                    A_id.append(k)
                    #del allvoteslist[k]
                    A  = True
            print("A:%s,A_id=%s" % (str(A),str(A_id)))
            # 方案B: 用kwtest检验多个样本是否来自同一分布
            # d,p=stats.kruskal(*allvoteslist)
            # if p < 0.05:
            #     B =True
            # print("B:" +str(B))
            #print("p: "+str(p))
            #方案C: 每个样本与其他所有样本作kstest,如果超过半数与该样本属于一个分布，则该样本为正常样本，否则为byztine样本
            #if different:
            for i in allrobotsIds_votes.keys():
                count = 0
                for j in allrobotsIds_votes.keys():
                    if i != j:
                        d,p = stats.kstest(allrobotsIds_votes[i],allrobotsIds_votes[j])
                        if p < 0.05:
                            count = count + 1
                            C=True
                if count > 7:
                    C_id.append(i)
            print("C:%s,C_id=%s" % (str(C),str(C_id)))
            #return C_id



            #若样本中存在不同分布，将所有样本分为两类，删除占少数的那类，再重复检验
            #若样本都属于同一分布，则检验完毕，返回均值和se
            if A:
                #continue
                X = np.array(mean_std)
                kmeans = KMeans(n_clusters=2, random_state=0).fit(X)
                # print(kmeans.labels_)
                # print(kmeans.cluster_centers_)
                #mean_std.remove()
                index = [i for i,j in enumerate(kmeans.labels_) if j == min(kmeans.labels_, key=list(kmeans.labels_).count)]
                #反向循环
                for i in reversed(index):
                    del mean_std[i]
                    #del dp[i]
                    byzantinerobot_id.append(allRobotIds[i])
                    del allvoteslist[i]
                    del allRobotIds[i]
                print("byzantinerobot_id: "+str(byzantinerobot_id))
                return byzantinerobot_id
            else:
                # print("success")
                # print("byzantineNumber: "+ str(len(allvoteslistcopy)-len(allvoteslist)))
                # print(byzantinerobot_id)
                # #print(mean_std)
                # print(total_mean)
                # print(total_std)
                # print("total_se: "+str(total_se))
                return A_id
    else:
        # allvotes=[]
        # for votes in allvoteslist:
        #     allvotes.extend(votes)
        # total_mean=np.mean(allvotes)
        # total_std=np.std(allvotes,ddof=1)
        # total_se=total_std/math.sqrt(len(allvotes))
        return byzantinerobot_id

#-----------------------------------------------------------------------------#
#									KMEANS									  #
#-----------------------------------------------------------------------------#

# 加载数据
def loadDataSet(fileName):  # 解析文件，按tab分割字段，得到一个浮点数字类型的矩阵
    dataMat = []              # 文件的最后一个字段是类别标签
    fr = open(fileName)
    for line in fr.readlines():
        curLine = line.strip().split('\t')
        fltLine = map(float, curLine)    # 将每个元素转成float类型
        dataMat.append(fltLine)
    return dataMat

# 计算欧几里得距离
def distEclud(vecA, vecB):
    return sqrt(sum(power(vecA - vecB, 2))) # 求两个向量之间的距离

# 构建聚簇中心，取k个(此例中为4)随机质心
def randCent(dataSet, k):
    n = shape(dataSet)[1]
    centroids = mat(zeros((k,n)))   # 每个质心有n个坐标值，总共要k个质心
    for j in range(n):
        minJ = min(dataSet[:,j])
        maxJ = max(dataSet[:,j])
        rangeJ = float(maxJ - minJ)
        centroids[:,j] = minJ + rangeJ * random.rand(k, 1)
    return centroids

# k-means 聚类算法
def kMeans(dataSet, k, distMeans =distEclud, createCent = randCent):
    m = shape(dataSet)[0]
    clusterAssment = mat(zeros((m,2)))    # 用于存放该样本属于哪类及质心距离
    # clusterAssment第一列存放该数据所属的中心点，第二列是该数据到中心点的距离
    centroids = createCent(dataSet, k)
    clusterChanged = True   # 用来判断聚类是否已经收敛
    while clusterChanged:
        clusterChanged = False;
        for i in range(m):  # 把每一个数据点划分到离它最近的中心点
            minDist = inf; minIndex = -1;
            for j in range(k):
                distJI = distMeans(centroids[j,:], dataSet[i,:])
                if distJI < minDist:
                    minDist = distJI; minIndex = j  # 如果第i个数据点到第j个中心点更近，则将i归属为j
            if clusterAssment[i,0] != minIndex: clusterChanged = True;  # 如果分配发生变化，则需要继续迭代
            clusterAssment[i,:] = minIndex,minDist**2   # 并将第i个数据点的分配情况存入字典
        # print(centroids)
        for cent in range(k):   # 重新计算中心点
            ptsInClust = dataSet[nonzero(clusterAssment[:,0].A == cent)[0]]   # 去第一列等于cent的所有列
            centroids[cent,:] = mean(ptsInClust, axis = 0)  # 算出这些数据的中心点
    return centroids, clusterAssment
