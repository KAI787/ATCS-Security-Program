def mostCommonWord(paragraph, banned):
    import re
    import collections
       
    ans=[]
    words = re.findall(r'[a-zA-Z]+', paragraph.lower())
    counter = {}
    max = 0
    for w in words:
        if w not in banned:
            if w in counter:
                counter[w] += 1
            else:
                counter[w] = 1
            if counter[w] > max:
                max = counter[w]
    for word in counter:
        if counter[word]==max:
            ans.append(word)
    return ans