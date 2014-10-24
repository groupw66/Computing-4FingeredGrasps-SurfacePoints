#ifndef __DAEUTILS_H_INCLUDED__
#define __DAEUTILS_H_INCLUDED__


/**
 * non-removable non-expandable circular list
 */
template<class T>
class SmallStore {
public:
    SmallStore(int maxSize = 100) {
        store = new T[maxSize];
        max = maxSize;
        clear();
    }

    ~SmallStore() {
        delete [] store;
    }

    void add(T data) {
      assert(firstFree < max);
      store[firstFree++] = data;
    }

    void clear() {
        firstFree = 0;
    }

    int getSize() {
        return firstFree;
    }

    T &operator [](int nIndex) const
    {
      assert((nIndex >= 0) && (nIndex < firstFree));
      return store[nIndex];
    }

    int first() { 
        assert(firstFree > 0);
        return 0; 
    }

    int last() { 
        assert(firstFree > 0);
        return firstFree-1; 
    }

    int succ(int itr) {
        return (itr + 1 == firstFree) ? -1 : itr+1;
    }

    int cyclic_succ(int &itr) {
        return (itr + 1 == firstFree) ? first() : itr+1;
    }

    int pred(int itr) {
        return (itr == 0) ? -1 : itr-1;
    }

    int cyclic_pred(int itr) {
        return (itr == 0) ? last() : itr-1;
    }

protected:
    T* store; 
    int firstFree,max;
};


#endif